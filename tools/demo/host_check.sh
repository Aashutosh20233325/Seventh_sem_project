#!/usr/bin/env bash
# 20 s read-only Windows host-memory check (typeperf via WSL interop). Prints "ok ..." and exits 0 only when the median
# Pages/sec is below 500 and the page file below 10 %; otherwise "high ..." / "bad ..." and exit 1. Heavy paging on the
# Windows host (e.g. browsers open) slows the simulation below the 0.35x readiness floor (docs/DEMO_CHECKLIST.md).
TP=/mnt/c/Windows/System32/typeperf.exe
out=$(timeout 40 $TP "\Memory\Available MBytes" "\Memory\Pages/sec" "\Paging File(_Total)\% Usage" -si 1 -sc 20 2>&1 | grep '^"[0-9]' | python3 -c '
import sys,csv,statistics as S
r=[x for x in csv.reader(sys.stdin) if len(x)==4]
if len(r)<15: print("bad", len(r), "samples"); sys.exit()
a,p,f=(S.median(float(x[i]) for x in r) for i in (1,2,3))
print("ok" if p<500 and f<10 else "high", f"avail_mb={a:.0f} pages_s_med={p:.1f} pagefile_pct={f:.1f}")')
echo "$out"
case "$out" in ok*) exit 0;; *) exit 1;; esac
