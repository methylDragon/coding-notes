# Functional Python 3 Crash Course

Author: methylDragon  
Contains a syntax reference for Python 3  
This time, we'll be going through Functions and File I/O!   
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=N4mEzFDjqtA

------

## Pre-Requisites

### Assumed knowledge

- The previous section!

### Good to know

- We're using Python 3 here! Python 2 has different syntax!
- If you have knowledge of computation structures like variables, functions, OOP, etc. it'll be easier
- But if not... https://www.youtube.com/playlist?list=PLGLfVvz_LVvTn3cK5e6LjhgGiSeVlIRwt
  - Or you can learn on the go!



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Functional C++ Syntax Reference](#2)    
   2.1   [Functions](#2.1)    
   2.2   [Local and Global Variables](#2.2)    
   2.3   [Recursive Functions](#2.3)    
   2.4   [File I/O](#2.4)    



## 1. Introduction <a name="1"></a>

We've gone through the basics of Python 3. Now let's throw in some functions and file interactions!



## 2. Functional Python 3 Syntax Reference <a name="2"></a>

### 2.1 Functions <a name="2.1"></a>

[go to top](#top)

Functions are chunks of code that can be called again after being declared!

They're a handy way to chunk up your code so you don't have to repeat what you declared!

```python
# Define a function using def

def add_numbers(first_num, second_num):
    added_numbers = first_num + second_num
    return added_numbers # Stop the function and output the value stored in added_numbers

# The function's name is add_numbers
# And it takes two parameters, first_num and second_num
# You can manipulate them inside the function, and return the result!

add_numbers(1,2) # Gives 3
add_numbers(4,5) # Gives 9
```

#### **Optional Inputs**

```python
# You can state OPTIONAL INPUTS as well!

# third_num is an optional input with default value 0
def add_more_numbers(first_num, second_num, third_num = 0): 
    added_numbers = first_num + second_num + third_num
    return added_numbers

add_more_numbers(1, 2) # Gives 3
add_more_numbers(4, 5) # Gives 9
add_more_numbers(4, 5, 1) # Gives 10! The optional value got overwritten with your input!
```

#### **Docstrings**

```python
# DOCSTRINGS
def my_function(some_parameters):
    '''
    You can include what's called a docstring in python 3 functions!
    The first string declared after the function declaration statement is the docstring
    
    Docstrings are used to help clarify your function's purpose and arguments
    As well as to provide other necessary help. Think of it as a help document!
    Triple-quotes are generally the convention even if the docstring is one line
    
    Docstrings can be accessed using  my_function.__doc__
    '''
    
# Docstring conventions from the PEP
# Multi-Line Docstring
def complex(real=0.0, imag=0.0):
    """Form a complex number.

    Keyword arguments:
    real -- the real part (default 0.0)
    imag -- the imaginary part (default 0.0)
    """
    if imag == 0.0 and real == 0.0:
        return complex_zero
    
# One-Line Docstring
def function(a, b):
    """Do X and return a list."""
```

#### **Yield**

```python
# If you want to define a function to generate items instead of returning a single one
# You can let the function have outputs without ending the function using Yield!

# It works just like Return, but it doesn't end the function!

def rawr():
    yield "raa"
    yield "rer"
    yield "rawr"

for item in rawr():
    print(item)
    
# Output:
# "raa"
# "rer"
# "rawr"

# Yield can do something cooler too!
# Combine it with next() to create an INFINITE ITERATABLE! (Without killing your memory)
# Yield computes only when it needs to!

def infinity():
    x = 0
    while True:
        yield x
        x += 1
        
infinity_forever = infinity()
while True:
    print(next(infinity_forever, end=", "))
    
# Prints
# 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 ......................
# At least, until your computer runs out of memory I guess?
```

#### **Default value pitfalls**

```python
# Python 3 supports default argument values as well!
# But there are pitfalls!
# If you pass a mutable type like a list, the same list will get altered each time you run the function, which means its default value won't be default anymore.

# If you really need to use a list
def myfunc(value = None):
    if value is None:
        value = []

# Read about it here: http://effbot.org/zone/default-values.htm

# If you really want to use default arguments, a good rule of thumb is to KEEP THEM IMMUTABLE
# None is a good one

def function_name(parameter1 = None):
    # bla bla bla
    
# The link I posted has this really good memoisation implementation
# I'll talk about memoisation in the advanced section, not here

def calculate(a, b, c, memo={}):
    try:
        value = memo[a, b, c] # return already calculated value
    except KeyError:
        value = heavy_calculation(a, b, c) # Running some expensive recursive function
        memo[a, b, c] = value # update the memo dictionary
    return value
```

> Note: Python 3 does not support function overloading like in C++. This is a consequence of it being a loosely typed language.



### 2.2 Local and Global variables <a name="2.2"></a>

[go to top](#top)

```python
# Let's look at the add_numbers function we had!
def add_numbers(first_num, second_num):
    added_numbers = first_num + second_num
    return added_numbers
  
# If you try to use the variable we defined inside the function outside of it, it won't work
print(added_numbers) # Will give you an error!
```

Variables declared inside a function are considered **local** and cannot be accessed outside of the function!

Variables declared outside a function are considered **global** and can be accessed everywhere!



### 2.3 Recursive Functions <a name="2.3"></a>

[go to top](#top)

Source: https://www.python-course.eu/recursive_functions.php

These are functions that call THEMSELVES. Trippy.

```python
# Let's calculate the nth Fibonacci number!
# [0, 1, 1, 2, 3, 5, 8, 13, 21, 34 ...]
# n = 0 returns 0
# Here, we're counting downwards, until we get n = 0 or n = 1 for each function call of fib()

def fib(n):
    if n == 0:
        return 0
    elif n == 1:
        return 1
    else:
        return fib(n-1) + fib(n-2) # The function calls itself here!
```

The last function calls to go on the stack are resolved first! USE THE STACK! Play some Magic: The Gathering! You'll understand!

> Note! An iterative version of this will be orders of magnitude faster the higher n is because the recursive version repeats a lot of work! UNLESS MEMOISATION IS USED! To be talked about in the advanced section!

```python
# FYI: This is the iterative version
# Here, we're counting upwards

def fibi(n):
    a, b = 0, 1
    for i in range(n):
        a, b = b, a + b
    return a
```



### 2.4 File I/O <a name="2.4"></a>

[go to top](#top)

File input output.

#### **Create, Open, and Write to a file!**

```python
# Create and Open a file!

# wb stands for file-mode: write in binary mode
my_file = open("rawr.txt", "wb") # Use "ab+" to Read and Append to file

# Fetch file mode
print(my_file.mode) # Returns wb

# Fetch file name
print(my_file.name) # Returns rawr.txt in this case

# Write to file
my_file.write(bytes("I'm Rawring inside this file!\n", 'UTF-8')) # UTF-8 is the encoding

# Close the file
my_file.close()
my_file.closed # Checks if a file is closed (returns True if yes, False if no)
```

#### **Read from the file!**

```python
import os # This is so we can delete the file later

# Open the file!
# r+ stands for file-mode: Read and Write
my_file = open("rawr.txt", "r+")

# Read entire file contents
text_in_my_file = my_file.read()

# Read line by line iteratively! Each call reads successive lines
# If you've reached the end of file, nothing is returned
# If it's a blank line, \n is returned
my_file.readline() # Course, rawr.txt only has one line though...

# You can also loop over the entire file!
for line in my_file:
    print(line, end="")

# Print contents
print(text_in_my_file) # Returns I'm Rawring inside this file!\n

# Delete a file!
os.remove("rawr.txt")
```

#### **tell() and seek()**

`tell()` returns an integer giving the current position of the cursor in the file (in bytes when in mode `b` or as a number otherwise)

`seek()` lets you change the position of the cursor! It takes two arguments, the second is optional

- `my_file.seek(offset[, whence])`

```python
# tell() returns an integer giving the current position of the cursor in the file 
# (in bytes when in mode b or as a number otherwise)
my_file.tell()

# seek() lets you change the position of the cursor! It takes two arguments, the second is optional
# my_file.seek(offset[, whence])
# Whence = 0 is default, which means absolute file positioning
# Whence = 1 means seek from current position
# Whence = 2 means seek from end of file
# Offset is how far to move
my_file.seek(0,0) # Returns you to the beginning of file
```

```python
# This is pretty nifty!
# You can use this to append to the end of a file in write mode!
# Course, you could always just use a+ ...

f = open("rawr.txt", "r+")

# Bring cursor to EOF
f.seek(0,2)

# Write!
f.write("\nRawr")

# Bring cursor to beginning of file
f.seek(0,0)

# Print the results!
print(f.read())

f.close()
```



#### **File Modes**

> - `r` for reading
> - `w` for writing
> - `r+` opens for reading and writing (cannot truncate a file)
> - `w+` for writing and reading (can truncate a file)
> - `rb+` reading or writing a binary file
> - `wb+` writing a binary file
> - `a+` opens for appending (All writes will occur at end of file regardless of cursor position)
>
> https://stackoverflow.com/questions/41406116/what-is-the-difference-between-rw-and-r

#### **Good to know**

```python
# It's good practice to open files using "with"
# This way, if you get any errors, the file will still close instead of you having to rely on garbage disposal

with open('rawr.txt', 'r+') as f:
    read_data = f.read()
    
# Other handy os module commands

# Rename()
os.rename(file_name, new_name)

# Mkdir()
os.mkdir("dir_name") # Like linux! Makes a directory

# Rmdir()
os.rmdir("dir_name") # Deletes the directory

# Change directory
os.chdir("dir_name") # Not like linux! :( But this is basically cd

# Get working directory
os.getcwd() # OS's version of linux's pwd
```

#### **For extra stuff**

Like.. say you want to read delimited data with json or something, or do more advanced writing,

Check: https://docs.python.org/3/tutorial/inputoutput.html



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

​    

------

 [![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)