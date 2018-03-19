#!/usr/bin/python2.7

import os

if __name__=='__main__':
    for _ in range(3):
        os.system('rosnode kill -a')
