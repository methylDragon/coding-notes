# Python 3 Crash Course

Author: methylDragon  
Contains a syntax reference for Python 3!  
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=N4mEzFDjqtA

------

## Pre-Requisites

### Good to know

- We're using Python 3 here! Python 2 has different syntax!
- If you have knowledge of computation structures like variables, functions, OOP, etc. it'll be easier
- But if not... https://www.youtube.com/playlist?list=PLGLfVvz_LVvTn3cK5e6LjhgGiSeVlIRwt
  - Or you can learn on the go!



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Basic Python 3 Syntax Reference](#2)    
   2.1   [Comments](#2.1)    
   2.2   [Importing Libraries](#2.2)    
   2.3   [Print](#2.3)    
   2.4   [Variables](#2.4)    
   2.5   [Arithmetic](#2.5)    
   2.6   [More Arithmetic](#2.6)    
   2.7   [Lists](#2.7)    
   2.8   [List Functions](#2.8)    
   2.9   [Tuples](#2.9)    
   2.10 [Dictionaries](#2.10)    
   2.11 [Dictionary Functions](#2.11)    
   2.12 [Conditionals](#2.12)    
   2.13 [Ternary Operators](#2.13)    
   2.14 [User Input](#2.14)    
   2.15 [For Loops](#2.15)    
   2.16 [Handy For Loop Functions and Keywords](#2.16)    
   2.17 [While Loops](#2.17)    
   2.18 [Strings](#2.18)    
   2.19 [String Functions](#2.19)    
   2.20 [Exception Handling](#2.20)    
3. [Reference Links](#3)  



## 1. Introduction <a name="1"></a>

If you are a beginner, Python is probably the way to go.

Python is easy to learn as a first language, and can be said to be 'elegant' in some views. It is very good for prototyping because it does a lot of work for you (like trash and memory management.) You also don't need to care so much about variable types because it automatically assigns them for you!

The trouble is, it's slow, so while it's good to learn and use for prototypes, you might want to pick up more languages as you go.

> Also, be aware that **capitalisation** and **whitespace** (like spaces and tabs) are VERY IMPORTANT in Python. 
>
> Whitespace errors (too many or too little before or after each line) can break your program, so make sure you set them correctly! (A single level of indentation in Python is 4 spaces.)

Here's the Python 3 style guide: https://www.python.org/dev/peps/pep-0008/#function-and-variable-names



## 2. Basic Python 3 Syntax Reference <a name="2"></a>

### 2.1 Comments <a name="2.1"></a>

[go to top](#top)

```python
# this is a comment

''' 
multi
line comment!
'''
```



### 2.2 Importing Libraries <a name="2.2"></a>

[go to top](#top)

```python
import <MODULE_NAME>
```

Sometimes you want to extend the functionalities of your Python installation by importing modules (libraries of functions other people have written.)

Common useful modules from the standard library that comes installed with Python include:

```python
import math
import decimal
import datetime
import time
import re # Regular Expressions, check my tutorial
```

More module advice:

https://stackoverflow.com/questions/1453952/most-useful-python-modules-from-the-standard-library

https://wiki.python.org/moin/UsefulModules



### 2.3 Print <a name="2.3"></a>

[go to top](#top)

You can display stuff on the console using print()

And manipulate it in cool ways!

```python
print("Rawr")
# Output: Rawr

print("Rawr","Rar") # Comma appends with a space between each by default!
# Output: Rawr Rar

print("Rawr","Rar",sep=", ") # You can change this default though!
# Output: Rawr, Rar

print("Rawr\nRar") # \n is a newline character!
''' Output:
Rawr
Rar
'''

print("a" * 5) # You can multiply!
# Output: aaaaa
```

If you want formatted output, use % or f

```python
# % example

dragon = "methylDragon"
print("%s %s %s" % ("Hi!", "I am", dragon))
# Output: Hi! I am methylDragon

# f example (Valid from Python 3.6 onwards)

dragon = "methylDragon"
stuff = "orchestral music"
print(f"Hi! I am {dragon} and I make {stuff}!")
# Output: Hi! I am methylDragon and I make orchestral music!

# .format example

dragon = "methylDragon"
stuff = "orchestral music"
print('{} makes {}!'.format(dragon, stuff))
# Output: methylDragon makes orchestral music

# Extra options
# There are a lot more! https://pyformat.info/
# When you're using {}, you can also add :<stuff> to add extra formatting options!
# {:<10} Left padding (align left)
# {:>10} Right padding (align right)
# {:^10} Centre padding (align centre)
# {:.5} Truncate string
# {:d} Int
# {:f} Float
# {:5d} Padded Int

```

```python
# Also note, each print statement automatically prints a newline

print("Hi")
print("Test")

''' Output:
Hi
Test
'''

# If you want to not do that
print("Hi", end="")
print("Test")

'''
Output:
HiTest
'''

# Basically, what end="" does, is replace the newline character with whatever you put in the quotes

print("Rawr", end=" I'm a dragon")

# Output: Rawr I'm a dragon
```



### 2.4 Variables <a name="2.4"></a>

[go to top](#top)

**Variables are like containers for data**

- Variables must start off with a letter, but can contain numbers, or underscores. (But they MUST start off with a letter.)
- They also cannot be keywords used in Python.
- Names are CASE SENSITIVE
- The Pythonic way of naming variables, is lowercase, separated by underscores. Or whatever the code was using. Keep it consistent and neat!

```python
# Assign variable values using =
my_variable = "Hi"
my_number = 6
my_float = 5.3
my_string = "\"I decided I wanted to put a quote in here"
# my_string has this weird ting \" because I wanted to put a quote in!
# If printed, it reads: "I decided I wanted to put a quote in here
# This is known as an escape character
my_multiline_quote = ''' this also
works wonders
isn\'t it great?!'''

# You don't have to declare your types! Python does it for you!
# Data types are good to know though! Here's some common ones
# string: List of characters
# int: Integer
# long: Integer with more 
# float: Floating Point Number (Decimal)
# double: Float with more memory space (More decimals)
# bool: True/False
# list: Ordered Array of same type
# tuple: Immutable Array of possibly different type
# dictionary: Keyed Array


# If you want to find out the type of a variable, use type()
type(my_number) # Returns <class "int">

# If you want to find its address, use id()
id(my_number) # Returns my_number's address
```

> Ok. I said variables are like containers for data, but that only helps beginners visualise it. In Python it's a little different. The variable names you define refer to objects that store values. But when you change the value, what happens is NOT that the value stored in the referred object changes, but rather, another object is created and the variable names becomes a reference to that new object.
>
> The names are essentially rebound! (I'm guessing this might be why the performance is so meh)
>
> See: https://stackoverflow.com/questions/10262920/understanding-pythons-call-by-object-style-of-passing-function-arguments



### 2.5 Arithmetic <a name="2.5"></a>

[go to top](#top)

```python
+ # Add: 3 + 2 equals 5
- # Subtract: 3 - 2 equals 1
* # Multiply: 3 * 2 equals 6
/ # Divide: 3 / 2 equals 1.5

** # Exponentiate: 3 ** 2 equals 9
// # Floor Divide: 3 // 2 equals 1 (Removes any decimals)
%  # Modulo: 3 % 2 equals 1 (Modulo returns the remainder!)
```

Arithmetic priority for the basic operators in Python is as follows:

```python
0. () # Like math!
1. **
2. * / % //
3. + -

# If operators are tied, go from left to right
```

There are others though!

https://www.tutorialspoint.com/python/operators_precedence_example.htm



### 2.6 More Arithmetic <a name="2.6"></a>

[go to top](#top)

```python
# There are some shortcuts as well!

my_int = 6
my_int += 10 # Means my_int = 6 + 10, my_int now equals 16
my_int -= 10 # Means my_int = 16 - 10
'''
You get the idea. Works for the other arithmetic operators too!
%= /= //= -= += = *=
'''

# Also, you can do some arithmetic on strings!
# It's called concatenation when you're joining them
string_one = "Hi"
string_two = "ya!"

string = string_one + string_two
# string now contains: Hiya!

# Sqrt() does a square root
sqrt(4) # Returns 2
```



### 2.7 Lists <a name="2.7"></a>

[go to top](#top)

Lists store multiple values of the same datatype. They're like 'boxes in memory'.

They are also ordered! Which means you can refer to elements inside them by index! (The index starts at 0)

(They're called arrays in most other programming languages. FYI.)

Example:

```python
# Define a list with []
# methylDragon has original orcehstral music on the
music_services = ["Spotify", "YouTube", "Google Play", "iTunes", "Deezer", "And More!"]

# Index 0 is "Spotify"
# Index 1 is "YouTube"
# Index 2 is "Google Play"
# So on and so forth

# Printing from index
print(music_services[0])
# Output: Spotify

# Printing slices (Prints up to but not including the second number)
# Slice syntax: list_name[startAt:endBefore:skip]
print(music_services[0:2])
# Output: ["Spotify", "YouTube"]

# Reassinging values
music_services[5] = "Rawr!"
print(music_services[5])
# Output: "Rawr!"

# You can create lists of lists, by the way!

numbers_a = [1, 2, 3]
numbers_b = [4, 5, 6]
numbers_c = [7, 8, 9]

numbers_list = [numbers_a, numbers_b, numbers_c]
# numbers_list is now [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

# Printing from nested lists
numbers_list[0][1]
# Output: 2
# The way to read it is, from the outermost layer, read index 0 (So the first list)
# Then, within that list, read index 1

# Joining lists
list_one = [1, 2]
list_two = [3, 4]
list_three = list_one + list_two
# list_three now contains: [1, 2, 3, 4]
```



### 2.8 List Functions <a name="2.8"></a>

[go to top](#top)

```python
dragon_list = ["Rawr", "Rar", "Raa"]

# Appending
dragon_list.append("Rer")
# dragon_list now contains: ["Rawr", "Rar", "Raa", "Rer"]
dragon_list.append("Pop_Me")
# dragon_list now contains: ["Rawr", "Rar", "Raa", "Rer", "Pop_Me"]

# Pop: Removes the last element
dragon_list.pop()
# dragon_list now contains: ["Rawr", "Rar", "Raa", "Rer"]

# Inserting into an index
dragon_list.insert(1, "Rrr")
# dragon_list now contains: ["Rawr", "Rrr", "Rar", "Raa", "Rer"]

# Removing
dragon_list.remove("Rrr")
# dragon_list now contains: ["Rawr", "Rar", "Raa", "Rer"]

# Delete an item
del dragon_list[3]
# dragon_list now contains: ["Rawr", "Rar", "Raa"]

dragon_list.sort() # Sort!
dragon_list.reverse() # Reverses order of list

# So if you want a reverse sort, do two calls. One to sort() and one to reverse()

# Return length of list
len(dragon_list) # This will return 3

# Return maximum (alphanumerically)
max(dragon_list) # Returns Raa

# Return minimum (alphanumerically)
min(dragon_list) # Returns Rawr

# Sum all values in the list
sum(dragon_list) # This won't work unless the list elements are numbers though...
```



### 2.9 Tuples <a name="2.9"></a>

[go to top](#top)

So we've done lists. Now let's do Tuples!

- Tuples can contain elements of different types
- But they are immutable! You cannot reassign values once you've set them

```python
# Define a tuple using ()
my_tuple = (1, 2, 3, 4)

# Convert a tuple into a list using list()
my_list = list(my_tuple)

# And a list back into a tuple using tuple()
my_tuple_again = tuple(my_list)

# These work the same as with list
len()
max()
min()
```



### 2.10 Dictionaries <a name="2.10"></a>

[go to top](#top)

A dictionary is made up of values with a **UNIQUE** key for each value! It's basically a keyed list.

They're also called maps

> You can't join them like you can with lists though!

```python
# Define a dictionary using {}
species_dictionary = {"Bob" : "Human",
                     "methylDragon" : "Dragon",
                     "Jane" : "Snake"}
# The things to the left of the colon are the keys, and the ones on the right are the values

# To call an entry by key
species_dictionary["methylDragon"] # Returns Dragon

# To reassign values (Think lists, but with keys instead of indexes!)
species_dictionary["Jane"] = "Human"
```



### 2.11 Dictionary Functions <a name="2.11"></a>

[go to top](#top)

```python
# List functions work!
del dictionary_name[<key>]
len()

# Get values
species_dictionary.get("methylDragon") # Returns Dragon
# This works the same as [], except, if no value is found, it defaults to NULL as opposed to throwing an error

# Get a list of keys
species_dictionary.keys()

# Get a list of values
species_dictionary.values()
```



### 2.12 Conditionals <a name="2.11"></a>

```python
== // equal to 
!= // NOT equal to
> // more than
< // less than
>= //  more than or equal to
<= // less than or equal to
```

> NOTE. "==" IS **NOT** "="
>
> == COMPARES
>
> = ASSIGNS VALUES

**Logical Operators**

```python
&& // AND
|| // OR
! // NOT
```

**Example IF, ELIF, ELSE**

```python
dragon_rating = 10

if dragon_rating < 10 : # If condition is fulfilled
	print("NEEDS MORE DRAGONS") # Do this
elif dragon_rating < 15 : # Else, If condition is fulfilled
	print("Good, but needs MORE") # Do this instead
else : # Else
	print("EXCELLENT") # Do this otherwise
```

```python
# ONE MORE! Let's do it with logical operators now

is_dragon = True
dragon_rating = 10

if (dragon_rating < 10 && is_dragon == True) :
    print("Oh no! Dragons shouldn't hate themselves!")
elif (dragon_rating < 10 && is_dragon != True) :
	print("NEEDS MORE DRAGONS")
else :
	print("Well ok then. Raa.")
```



### 2.13 Ternary Operators <a name="2.13"></a>

[go to top](#top)

Use these if you want to look cool!

Example

```c++
// a if condition else b

print("True") if 5 == 5 else print("False")
```

> I prefer C++'s way though... 
>
> (condition) ? true : false;
>
> But eh



### 2.14 User Input <a name="2.14"></a>

[go to top](#top)

```python
# Use input() !!
input_variable = input("Input here: ") # The console will display Input Here: 

# You can also convert the input to whatever you need!
float_input = float(input("Put your float here: "))
```

> NOTE: DO NOT USE eval() UNLESS YOU KNOW WHAT YOU'RE DOING
>
> It lets the user run code directly in your program. That's what eval() does-- run whatever was eval()'ed as if it were code!



### 2.15 For Loops <a name="2.15"></a>

[go to top](#top)

Python does for loops in a way I don't really prefer... (I learnt them doing C++) But eh, I guess it can be a little more intuitive...?

If you really want to go advanced and ask about how the for loop really works, see:

https://www.codementor.io/sheena/python-generators-and-iterators-du1082iua

```python
# Iterate FOR each element in a list
# Output: Rawr Rar Raa Rer
my_list = ["Rawr", "Rar", "Raa", "Rer"]
for roar in my_list:
    print(roar, end=" ")

# Iterate FOR some range of numbers
# Output: 0 1 2 3 4 5 6 7 8 9
for element in range(10): # You can call the element anything, some people even use _ !!
    print(element, end=" ")
    
# You can use the iterations to do stuff with them!
# Output: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
count = 0
for _ in range(10):
    count++
    print(count, end=", ")
```



### 2.16 Handy For Loop Functions and Keywords <a name="2.16"></a>

[go to top](#top)

**range()**

```python
range(5) # Returns [0, 1, 2, 3, 4]
range(3,6) # Returns [3, 4, 5]
range(4,10,2) # Returns [4, 6, 8]
range(0,-10,-2) # Returns [0, -2, -4, -8]

# If you want to turn a list's elements into a list of indexes
list_name = ["rawr", "raa", "rer"]
range(len(list_name)) # Returns [0, 1, 2]
```

**Slicing**

```python
# Remember that you can slice lists!
# list_name[startAt:endBefore:skip]
my_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
for _ in my_list[::2]
	print(_,end=" ")
    
# Output: 1 3 5 7 9
```

**enumerate()**

```python
rawr_list = ["Rawr", "Rar", "Rer", "Raa"]

# Enumerate() returns a list of tuples, with an increasing counter
# Output: 0 Rawr, 1 Rar, 2 Rer, 3 Raa,
for num, rawr in enumerate(rawr_list):
	print(num, rawr, end=", ")

# You can also tweak the starting number!
# Output: 5 Rawr, 6 Rar, 7 Rer, 8 Raa,
for num, rawr in enumerate(rawr_list,5):
    print(num, rawr, end=", ")
```

**zip()**

```python
rawr_list = ["Rawr", "Rar", "Rer", "Raa"]
num_list = [1, 2, 3, 4]

# Zip() lets you iterate through multiple lists!
# Output: Rawr 1, Rar 2, Rer 3, Raa 4,
for x, y in zip(rawr_list, num_list):
    print(x, y, end=", ")
```

**continue and break**

```python
num_list = [1, 2, 3, 4, 5]

# Continue skips the rest of the execution for the current loop, but continues the loop
# Output: 1 Check! 2 3 Check! 4 Check! 5 Check!
for x in num_list:
    print(x, end=" ")
    if x == 2:
        continue
    print("Check!", end=" ")
    
# Break just ends the entire loop then and there
# Output: 1 Check! 2
for x in num_list:
    print(x, end=" ")
    if x == 2:
        break
    print("Check!", end=" ")
```



### 2.17 While loops <a name="2.17"></a> 

[go to top](#top)

Use these when you don't know ahead of time when this loop is going to end.

```python
import random

random_num = random.randrange(0,100) # Generate a random number between 0 and 100

while(random_num != 15): # This while loop will stop once random_num becomes 15
    print(random_num)
    random_num = random.randrange(0,100)
```

It'll print until the condition is no longer fulfilled! So in this case we'll print random numbers until random_num assumes the value 15.

Here's another example!

```python
i = 0

while(i <= 20): # This while loop will stop once i becomes 21
    print(i)
```

>  You can also use all the cool stuff listed in the previous section with while loops!
>
>  Even break and continue!



### 2.18 Strings <a name="2.18"></a>

[go to top](#top)

Let's play around with strings!

```python
rawr_string = "I am methylDragon, and while I can talk, I also rawr!"

# Slice strings!
rawr_string[0:4] # Returns first 4 characters! "I am"
rawr_string[-5:] # Returns last 5 characters! "rawr!"
rawr_string[:-5] # Returns everything UNTIL the last 5 characters!

# Join strings
rawr_string[:-5] + "roar!"
# Output: I am metyhlDragon, and while I can talk, I also roar!
```

**Formatted Strings**

```python
# % example

dragon = "methylDragon"
print("%s %s %s" % ("Hi!", "I am", dragon))
# Output: Hi! I am methylDragon

# f example (Valid from Python 3.6 onwards)

dragon = "methylDragon"
stuff = "orchestral music"
print(f"Hi! I am {dragon} and I make {stuff}!")
# Output: Hi! I am methylDragon and I make orchestral music!

```



### 2.19 String Functions <a name="2.19"></a>

[go to top](#top)

```python
rawr_string = "I am methylDragon, and while I can talk, I also rawr!"

# Capitalise the first letter of the string
rawr_string.capitalize()

# Return the index of a found string (Case-Sensitive)
rawr_string.find("rawr") # Returns 48

# Check to see if everything is alphabetical
rawr_string.isalpha() # Returns False (we have punctuation!)

# Check to see if everything is numeric
rawr_string.isalnum() # Returns False (because, of course)

# Find length
len(rawr_string)

# Replace
rawr_string.replace("methylDragon", "a dragon")
# Let's undo that
rawr_string.replace("a dragon", "methylDragon")

# Strip away all whitespaces
rawr_string.strip()

rawr_string = "I am methylDragon, and while I can talk, I also rawr!" # Let's reset

# Split a string by some delimiter
rawr_string.split(",")
# Output: ['I am methylDragon', 'and while I can talk', ' I also rawr!']
```



### 2.20 Exception Handling <a name="2.20"></a>

[go to top](#top)

By far the most important concept for debugging purposes! Try to always use them!

Alternatively, if you know where a program might throw **exceptions** (i.e. errors), you can code in handlers for them if you know they can occur but it's either by design or because it might be user errors, etc. !

**Try-Except**

```python
# Exception handling is done mostly with try-except blocks
try:
    # Some code
except <error_one>:
    # Run this code ONLY if an exception for <error_one> is thrown
except <error_two> as err:
    print(err) # Print the error ONLY if <error_two> is thrown
except:
    # Wildcard error. Run this code ONLY if an exception unaccounted for is thrown
```
**Else**

```python
# You can also use else in try-except blocks!
try:
    # Some code
except:
    # Wildcard error handler
else:
    # Run if no errors were thrown AFTER running the code protected by the try block
```
**Finally**

```python
# You use the keyword finally to run code regardless of whether an exception was thrown or not
try:
    # Some code
except:
    # Wildcard error handler
else:
    # Run if NO ERRORS were thrown
finally:
    # Run before exiting the entire try block, whether errors were thrown or not
```

**Raise**

```python
# You can raise an exception, forcing it occur
raise NameError('some_strign')

# Let's say you do this in the try block
# The first raise won't be thrown, so you can reraise it using just raise
try:
    raise NameError('some_strign')
except:
    raise
```

**With**

```python
# This isn't exactly exception handling, and I mention it again in file I/O
# But... This is still useful

with open("myfile.txt") as f:
    # Some code
    
# Because the file was opened, an object was created
# With ensures that the file will be closed and cleaned from memory when appropriate
```



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```
