# ROS Tips and Tricks (WORK IN PROGRESS)

Author: methylDragon  
Just a collection of useful things I've found    

------

## Pre-Requisites

- A system with Ubuntu 16.04 installed (no other versions!)
- Linux
- Python 3 and/or C++



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Convenience Tips](#2)    
   2.1   [Linux Aliases](#2.1)    



## 1. Introduction <a name="1"></a>

Everyone loves being ~~lazy~~ efficient!



## 2. Convenience Tips <a name="2"></a>

### 2.1 Linux Aliases <a name="2.1"></a>

[go to top](#top)

Ever wish you could just swap your stated ROS_MASTER_URIs and ROS_HOSTNAMEs **without** having to go `$ sudo nano ~/.bashrc` or typing out the entire line?

**Aliases** have you covered! (For more than just those purposes!)

Inside ~/.bashrc:

```bash
alias command_name="<run this as if you ran it in terminal>" 
# Don't add spaces between the = sign and the words, it'll break it

# So for the IP change example:
alias rosinit_local="export ROS_HOSTNAME = localhost && export ROS_MASTER_URI = http://localhost:11311"
```

Now, you can do this!

```shell
$ rosinit_local

# Is equivalent to running
$ export ROS_HOSTNAME = localhost && export ROS_MASTER_URI = http://localhost:11311
```

And your IPs will be set accordingly! Easy!




```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\
```

​    

------

[![Yeah! Buy the DRAGON a COFFEE!](../../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)