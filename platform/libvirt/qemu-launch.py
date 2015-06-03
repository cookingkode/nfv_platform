#!/usr/bin/python
#####################################################################
# Copyright Jyotiswarup Raiturkar
#####################################################################
#####################################################################
# This script is designed to modify the call to the QEMU emulator 
# to support userspace vhost when starting a guest machine through 
# libvirt with vhost enabled. The steps to enable this are as follows 
# and should be run as root:
#
# 1. Place this script in a libvirtd's binary search PATH ($PATH)
#    generally this is /usr/bin
#   
# 2. Ensure that the script has the same owner/group and file 
#    permissions as the QEMU binary
#
# 3. Update the VM xml file using "virsh edit VM.xml"
#
#    	Set the emulator path contained in the 
#		<emulator><emulator/> tags
#
#    	e.g replace <emulator>/usr/bin/qemu-kvm<emulator/>
#        with    <emulator>/usr/bin/qemu-luanch.py<emulator/>
#
#####################################################################


#############################################
# Configuration Parameters
#############################################
#Path to QEMU binary 
emul_path = "/usr/local/bin/qemu-system-dpdk-x86_64"
#emul_path = "/usr/bin/qemu-system-x86_64"

#List of additional user defined emulation options. These options will
#be added to all Qemu calls 
emul_opts_user = []

#List of additional user defined emulation options for vhost only.
#These options will only be added to vhost enabled guests 
emul_opts_user_vhost = []

#For all VHOST enabled VMs, the VM memory is preallocated from hugetlbfs
# Set this variable to one to enable this option for all VMs
use_huge_all = 0

#Instead of autodetecting, override the hugetlbfs directory by setting
#this variable
hugetlbfs_dir = ""

#############################################


#############################################
# ****** Do Not Modify Below this Line ******
#############################################

import sys, os, subprocess, signal 



#############################################
# Find the system hugefile mount point.
# Note:
# if multiple hugetlbfs mount points exist
# then the first one found will be used
############################################# 
def find_huge_mount():

    if (len(hugetlbfs_dir)):
        return hugetlbfs_dir

    huge_mount = ""

    if (os.access("/proc/mounts", os.F_OK)):
        f = open("/proc/mounts", "r")
        line = f.readline()
        while line:
            line_split = line.split(" ")
            if line_split[2] == 'hugetlbfs':
                huge_mount = line_split[1]                
                break
            line = f.readline()
    else:
        print "/proc/mounts not found"
        exit (1)

    f.close
    if len(huge_mount) == 0:
        print "Failed to find hugetlbfs mount point"
        exit (1)    

    return huge_mount


def get_ivshm_config(name):
    file = "/tmp/.ivshmem_qemu_cmdline_" + name
    try:
         f = open(file).read()
    except: 
         f = ""
    return f

def get_dpdk_config():
    return " -c 0x4 -n 4 --proc-type=secondary -- "

def process_cleanup(signum, frame):
    c = "kill -- -\""+str(os.getpgid(os.getpid()))+"\" "
    print c
    os.system(c)
    sys.exit(0)

for sig in [signal.SIGTERM, signal.SIGINT, signal.SIGHUP, signal.SIGQUIT]:
    signal.signal(sig, process_cleanup )


#############################################
# Main
#############################################
def main():
    new_args = []
    num_cmd_args = len(sys.argv)
    emul_call = ''
    mem_prealloc_set = 0
    mem_path_set = 0 
    num = 0;

    emul_call = ""
    ivshmem_config = ""
    dpdk_config = get_dpdk_config()
    dpdk_runtime_config = ""

     
    echo_str = "echo \"" + " ".join(sys.argv) + "\" > /tmp/emul_args"
    os.system(echo_str)


    #parse the parameters
    while (num < num_cmd_args):
        arg = sys.argv[num]
        print arg
        
        if arg == '-name':
            print "got name !"
            ivshmem_config += get_ivshm_config(sys.argv[num+1])
    	    # copy DPDK runtime data
            share_path = "/tmp/share/"+ sys.argv[num+1] 
            try:
                os.makedirs(share_path)
	    except:
                print " directory exists "
	    cmd_str = "cp -a /var/run/.rte_* " + share_path
            os.system(cmd_str)
            dpdk_runtime_config = " -drive file=fat:" + share_path + ",if=scsi " 
            print ivshmem_config
            print dpdk_config

        num+=1

    #Set Qemu binary location 
    emul_call+=emul_path
    emul_call+=" "
    emul_call+= dpdk_config
    emul_call+= " ".join(sys.argv[1:])
    emul_call+=" "
    emul_call+= dpdk_runtime_config
    emul_call+=" "
    emul_call+="-cpu host "
    emul_call+=" "
    emul_call+= ivshmem_config

    #Call QEMU 
    echo_str = "echo \"" + emul_call + "\" > /tmp/emul_call"
    os.system(echo_str)
    print "\n\n" + emul_call
    subprocess.call(emul_call, shell=True)




if __name__ == "__main__":
    main()

