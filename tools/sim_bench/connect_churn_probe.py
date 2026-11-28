#!/usr/bin/env python3
"""RSS growth of the simulator per Remote API connection (local-only; needs a simulator from tools/sim.sh start).
The bridges drop the connection and reconnect after a timeout (sim_client.py: RemoteAPIClient + require('sim') each time).
Usage: connect_churn_probe.py [N=100] [--close]   --close also closes the client socket and context like close_last()."""
import os, subprocess, sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

pid = int(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '.sim', 'sim-23000.pid')).read())


def rss_mb():
    tot = 0
    for p in [str(pid)] + subprocess.run(['pgrep', '-P', str(pid)], capture_output=True, text=True).stdout.split():
        o = subprocess.run(['ps', '-o', 'rss=', '-p', p], capture_output=True, text=True).stdout.split()
        tot += int(o[0]) if o else 0
    return tot / 1024.0


n = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 100
close = '--close' in sys.argv
m0 = rss_mb()
for i in range(n):
    c = RemoteAPIClient(); sim = c.require('sim'); sim.getSimulationTime()
    if close:
        c.socket.close(0); c.context.term()
    if (i + 1) % max(1, n // 5) == 0:
        print('connections %d rss %.0f MB (%.2f MB per connection)' % (i + 1, rss_mb(), (rss_mb() - m0) / (i + 1)), flush=True)
