# This script prints out the first network interface name that has IBSS compatibility

import subprocess
import re


# Finds phy that supports ibss
def find_phy(iw_out: [str]):
    last_phy = None

    for line in iw_out:
        # If start of new phy
        match = re.match(r'Wiphy (\w*)', line)
        if match:
            raw_phy = match.group(1)
            # Add #
            last_phy = f'phy#{raw_phy[-1]}'
            continue

        # Make sure the ibss is in the interface modes, not the frame types
        interface_mp = re.match(r'.*\* IBSS', line)
        not_interface_mp = re.match(r'.*\* IBSS:', line)

        if interface_mp and not not_interface_mp:
            return last_phy


# Finds dev from phy
def find_dev(phy: str, dev_out: [str]):
    phy_found = False

    for line in dev_out:
        # Find phy we are in
        if re.match(phy, line):
            phy_found = True
            continue
        elif re.match('phy#.', line) and phy_found:
            print(f'No interface set to {phy}!')
            return None

        # In phy we want, find interface name
        if phy_found:
            match = re.match(r'.*Interface (\w*)', line)
            if match:
                return match.group(1)


iw_out = subprocess.getoutput('iw phy').splitlines()
dev_out = subprocess.getoutput('iw dev').splitlines()

print(find_dev(find_phy(iw_out), dev_out))
