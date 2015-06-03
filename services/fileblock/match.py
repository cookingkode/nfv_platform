#! /usr/bin/python
#
#   Copyright(c) 2010-2014 Jyotiswarup Raiturkar 
#   All rights reserved.

import os
import string

def my_ceil(x,y):
    return x/y + (x%y >0)

def chunk_file(file, chunk_sz):
    chunks = {}
    with open(file, 'r+b') as f:
        file_sz = os.stat(file).st_size
        for i in range(my_ceil(file_sz,chunk_sz)):
            chunks[i] = f.read(chunk_sz)
            f.seek(chunk_sz*(i+1))

    return chunks

test_file = "1.txt"
chunk_size = 50
pattern = "jyotiswarup"
p_len = len(pattern)

source = chunk_file(test_file, chunk_size)

# client
def find_best_suffix(t, p):
    pan_len = len(p)
    did_match  = 0
    for i in range(pan_len):
        if did_match == 1 :
            return i -1
        did_match  = 1
        for j in range(pan_len-i):
            if(t[j] != p[j+i]):
                did_match = 0
                break

    return -1


def find_best_match(t,p):
    ret = string.find(t, p)
    if ret == -1 :
        suffix = find_best_suffix(t,p) #no chars in p to skip at top
        prefix = find_best_suffix(p, t[len(t)-len(p):]) #no chars matched at bottomm
        if suffix<0 and prefix<0 :
            return (-1,0,0)

        return (1, suffix, prefix)
    else:
        print "\n\nexact match\n\n" + str(ret)
        return (0,0,0)


prev_match = [0,0]
def final(key, val):
    value = val[0]
    #print key,value[0],value[1]
    if value[0] == 0:
        print "full match in chunk" + str(key)
    elif value[0] == 1:
        if value[1] > 0:
            if (prev_match[1] + value [1]) == p_len :
                print "pattern strides chunks " + str(key-1) + " and "  + str(key)
        if value[2] > 0:
            prev_match[1] = value[2]


def mapfn(key, value):
    p = find_best_match(value,pattern)
    yield  key,p

def reducefn(key, value):
    return value 
