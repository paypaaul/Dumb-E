"""
End-to-end smoke test: boots the firmware in QEMU and drives serial protocol v0.

Usage, from the firmware/ directory (ESP-IDF environment exported, QEMU installed with
`python $IDF_PATH/tools/idf_tools.py install qemu-xtensa`):
    idf.py -B build-qemu -DSDKCONFIG=build-qemu/sdkconfig \
        -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;test/qemu/sdkconfig.qemu" build
    python test/qemu/smoke_test.py build-qemu
"""
import binascii, os, re, subprocess, sys, threading, time, queue

BUILD = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "build-qemu")
SP = BUILD
flash = os.path.join(SP, "qemu_flash.bin")
efuse = os.path.join(SP, "qemu_efuse.bin")

subprocess.check_call([sys.executable, "-m", "esptool", "--chip", "esp32", "merge-bin", "--pad-to-size", "4MB",
                       "-o", flash, "@flash_args"], cwd=BUILD, stdout=subprocess.DEVNULL)
with open(efuse, "wb") as f:
    f.write(binascii.unhexlify("00000000000000000000000000800000000000000000100000000000000000000000000000000000"
                               + "0" * 160 + "00000000"))

cmd = ["qemu-system-xtensa", "-M", "esp32", "-m", "4M", "-drive", f"file={flash},if=mtd,format=raw",
       "-drive", f"file={efuse},if=none,format=raw,id=efuse", "-global", "driver=nvram.esp32.efuse,property=drive,value=efuse",
       "-global", "driver=timer.esp32.timg,property=wdt_disable,value=true",
       "-nic", "user,model=open_eth",
       "-nographic", "-serial", "mon:stdio"]
p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
lines = queue.Queue()
log = open(os.path.join(SP, "qemu_serial.log"), "w")

def reader():
    buf = b""
    while True:
        ch = p.stdout.read(1)
        if not ch:
            break
        buf += ch
        if ch == b"\n":
            s = buf.decode(errors="replace").rstrip("\r\n")
            log.write(s + "\n"); log.flush()
            lines.put(s)
            buf = b""
threading.Thread(target=reader, daemon=True).start()

def wait_for(pattern, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        try:
            s = lines.get(timeout=0.5)
        except queue.Empty:
            continue
        m = re.search(pattern, s)
        if m:
            return s
    raise TimeoutError(pattern)

def send(c, timeout=10):
    p.stdin.write((c + "\r").encode()); p.stdin.flush()
    s = wait_for(r"(^|>|\s)(ok|err \d+)\b", timeout)
    s = s[s.find("ok") if "ok" in s and (s.find("err ") < 0 or s.find("ok") < s.find("err ")) else s.find("err "):]
    print(f"> {c}\n  {s}")
    return s

def status():
    return send("status")

def wait_idle(timeout=60):
    end = time.time() + timeout
    while time.time() < end:
        s = status()
        if "moving=0" in s:
            return s
        time.sleep(1)
    raise TimeoutError("still moving")

failures = []
def check(cond, what):
    print(("  PASS " if cond else "  FAIL ") + what)
    if not cond:
        failures.append(what)

try:
    wait_for(r"robot: ready", 60)
    time.sleep(2)
    send("version")
    s = send("config"); check(s.startswith("ok"), "config ends with ok")
    s = status(); check("state=DISABLED" in s and "ref=0" in s, "boots DISABLED and unreferenced")
    s = send("movej 10 90 -90 0 0"); check(s.startswith("err 1") or "err 3" in s or "err 2" in s, "move refused while disabled")
    send("enable")
    s = send("movej 10 90 -90 0 0"); check("err 3" in s, "absolute move refused while not referenced")
    s = send("zero"); check(s.startswith("ok"), "zero at park pose")
    s = status(); check("state=READY" in s and "steps=0,32000,-32000,0,0" in s, "park pose -> steps 0,32000,-32000,0,0")
    s = send("movej 10 80 -60 20 45 -v 100 -a 100"); check(s.startswith("ok"), "movej accepted")
    s = wait_idle()
    # expected steps: deg * 32000/90
    exp = [round(d * 32000 / 90) for d in (10, 80, -60, 20, 45)]
    check("steps=" + ",".join(str(e) for e in exp) in s, f"final steps exactly {exp}")
    check("underruns=0" in s and "rejected=0" in s, "no underruns / rejected segments")
    s = send("movej 200 80 -60 20 45"); check("err 5" in s, "joint limit rejected")
    s = send("movep 200 0 150 -90 0"); print("  (movep)", s)
    if s.startswith("ok"):
        s = wait_idle(); check("tcp=200.00,0.00,150.00,-90.00,0.00" in s or "tcp=200.0" in s, "movep reaches TCP")
    s = send("jog 1 -5"); check(s.startswith("ok"), "jog accepted"); wait_idle()
    # stop in the middle of a long move
    send("movej -100 150 -120 90 -170 -v 20")
    time.sleep(1.0)
    s = send("stop"); check(s.startswith("ok"), "stop accepted")
    s = wait_idle(); check("underruns=0" in s, "controlled stop without underrun")
    q = re.search(r" q=([^ ]+)", s).group(1).split(",")
    steps = re.search(r"steps=([^ ]+)", s).group(1).split(",")
    exp = [round(float(x) * 32000 / 90) for x in q]
    check(all(abs(int(a) - b) <= 1 for a, b in zip(steps, exp)), "stepgen position matches commanded q after stop")
    # estop
    send("movej 0 90 -90 0 0 -v 20"); time.sleep(0.5)
    s = send("estop"); s = status(); check("state=ESTOP" in s, "estop latched")
    s = send("movej 0 90 -90 0 0"); check("err 8" in s, "moves refused during estop")
    wait_idle()
    s = send("reset"); s = status(); check("state=READY" in s, "reset -> READY")
    s = send("park -v 100"); wait_idle(); s = status()
    check("steps=0,32000,-32000,0,0" in s, "park returns exactly to park steps")
    s = send("grip 30"); check(s.startswith("ok"), "gripper command")
    s = send("fk 0 90 -90 0 0"); check("x=210.000" in s and "z=250.000" in s, "fk of park pose")
    s = send("ik 210 0 250 0 0"); check(re.search(r"q=-?0.00\d,90.00\d,-90.00\d,-?0.00\d,-?0.00\d", s) is not None, "ik of park pose (elbow up)")
    s = status(); print("  final:", s)
    s = send("disable"); s = status(); check("state=DISABLED" in s and "ref=0" in s, "disable clears reference")
finally:
    p.kill()
print("\nFAILURES:" if failures else "\nALL CHECKS PASSED", failures if failures else "")
sys.exit(1 if failures else 0)
