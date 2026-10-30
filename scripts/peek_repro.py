#!/usr/bin/env python3
"""抽样打印离线复现 trace 的 D 段 / R 段起始逐拍量。"""
import csv
import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/e2_repro.csv'
step = int(sys.argv[2]) if len(sys.argv) > 2 else 4
rows = list(csv.DictReader(open(path)))
print(f'{len(rows)} frames;  cols: t e_meas e_true phi kappa kref k_pp la_d rem gear flip')
hdr = ['t', 'e_meas', 'e_true', 'phi', 'kappa', 'kref', 'k_pp', 'la_d', 'remaining', 'gear', 'flip']
last_gear = None
for k, r in enumerate(rows):
    if k % step and r['gear'] == last_gear:
        continue
    last_gear = r['gear']
    phi_d = float(r['phi']) * 57.2958
    print(' '.join([
        f"t={float(r['t']):6.2f}",
        f"e_m={float(r['e_meas']):+.3f}",
        f"e_t={float(r['e_true']):+.3f}",
        f"phi={phi_d:+6.2f}d",
        f"kap={float(r['kappa']):+.3f}",
        f"krf={float(r['kref']):+.3f}",
        f"kpp={float(r['k_pp']):+.3f}",
        f"lad={float(r['la_d']):5.2f}",
        f"rem={float(r['remaining']):6.2f}",
        f"g={r['gear']}",
        f"f={r['flip']}"]))
