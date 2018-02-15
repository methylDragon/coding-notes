# Advanced Python 3 Crash Course

Author: methylDragon  
Contains an advanced syntax reference for Python 3  
This time, we'll be going through many many (mostly unrelated) Python 3 coding concepts!  
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=N4mEzFDjqtA

------

## Pre-Requisites

### Assumed knowledge 

- Gone through the all preceding parts of the tutorial


### Good to know

- We're using Python 3 here! Python 2 has different syntax!
- If you have knowledge of computation structures like variables, functions, OOP, etc. it'll be easier
- But if not... https://www.youtube.com/playlist?list=PLGLfVvz_LVvTn3cK5e6LjhgGiSeVlIRwt
  - Or you can learn on the go!



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Advanced Python 3 Syntax and Concepts](#2)    
   2.1   [if \_\_name__ == "\_\_main__":](#2.1)    
   2.2   [Lambda, Filter(), Reduce(), and Map()](#2.2)    
   2.3   [List Comprehensions](#2.3)    
   2.4   [*args and **kwargs](#2.4)    
   2.5   [Child Class Declaration](#2.5)    
   2.6   [Virtual Methods and Polymorphism](#2.6)    




## 1. Introduction <a name="1"></a>

Ok. We've gone through all the basic stuff and simple syntax.

Now let's dive into the deep end. Very little hand holding here, but lots of useful concepts, and some pretty powerful stuff! 

> I can't find a way to organise these, so just look through what looks interesting. I'll try to write pre-requisite concepts so you can have things to Google or refer to, but we're going to go pretty fast.

## 2. Advanced Python 3 Syntax and Concepts <a name="2"></a>

### 2.1 if \_\_name__ == "\_\_main__": <a name="2.1"></a>

[go to top](#top)

Looks intimidating at first, but it actually means something really simple.

Each time Python 3 opens and read code, it sets several special (global) variables before executing said code. One of these is the `__name__` variable.

If the Python 3 main interpreter is running the program, (i.e. that the program is being run directly instead of through an import statement), `__name__` is set to `"__main__"`

```python
if __name__ == "__main__": # There we are!
	print("We're running the program directly!")
else:
	print("We're running the program indirectly! It's imported!")
```



### 2.2 Lambda, Filter(), Reduce(), and Map() <a name="2.2"></a>

[go to top](#top)



### 2.3 List Comprehensions <a name="2.3"></a>

[go to top](#top)

>  We just went through lambda functions. List comprehensions are **generally** viewed as preferable to use if you can help it than lambda, map, filter, reduce, because they're easier to understand and less finicky!

This is how list comprehensions are used

Remember lists?

```python
my_list = [1, 2, 3, 4, 5]
```

Ever thought they looked weird, or finicky? Ever wanted to generate them automatically like you could in a for loop without using random appends?

WELL NOW YOU CAN!

```python
# List comprehensions allow you to define lists much like the way
# Mathematicians do!

# You can do this with math
exponent_list = [x**2 for x in range(10)]
# exponent_list: [0, 1, 4, 9, 16, 25, 36, 49, 64, 81]

# Or using conditionals to filter results!
# Here it filters it only if the exponents are divisible by 2
exponent_list_filtered = [x for x in exponent_list if x % 2 == 0]
# exponent_list_filtered: [0, 4, 16, 36, 64]

# Or do successive bool tests!
bool_list = [(x % 2 == 0) for x in exponent_list]
# bool_list: [True, False, True, False, True, False, True, False, True, False]
```

Basically, they're stupidly useful, and super elegant. I love them!

```python
# Here's a bunch of more complicated list comprehensions!
# Source: http://www.secnetix.de/olli/Python/list_comprehensions.hawk

# This one uses two list comprehensiosn together
noprimes = [j for i in range(2, 8) for j in range(i*2, 50, i)]
primes = [x for x in range(2, 50) if x not in noprimes]
# primes: [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47]
```

```python
# You can also use it with non numbers!

words = 'The quick brown fox jumps over the lazy dog'.split()
# words: ['The', 'quick', 'brown', 'fox', 'jumps', 'over', 'the', 'lazy', 'dog']

stuff = [[w.upper(), w.lower(), len(w)] for w in words]
for i in stuff:
	print i
    
'''
['THE', 'the', 3]
['QUICK', 'quick', 5]
['BROWN', 'brown', 5]
['FOX', 'fox', 3]
['JUMPS', 'jumps', 5]
['OVER', 'over', 4]
['THE', 'the', 3]
['LAZY', 'lazy', 4]
['DOG', 'dog', 3]
'''

# Course, the equivalent can be used with lambda functions
# But it's less comprehensible (that was not meant to be a pun)
stuff = map(lambda w: [w.upper(), w.lower(), len(w)], words)
for i in stuff:
	print i
```

> You cannot use list comprehensions when the construction rule is too complicated to be expressed with "for" and "if" statements, or if the construction rule can change dynamically at runtime.  
>
> In this case, you better use `map()` and / or `filter()` with an appropriate function.
>
> Of course, you can combine that with list comprehensions.
>
> Source: http://www.secnetix.de/olli/Python/list_comprehensions.hawk



### 2.4 *args and **kwargs <a name="2.4"></a>

[go to top](#top)

Source: https://pythontips.com/2013/08/04/args-and-kwargs-in-python-explained/

Well. Technically it's *whatever \*\*whatever, but convention is *args and **kwargs.

**\*args**

*args and **kwargs can be generally seen to be used in function definitions. They both allow for you to pass a variable (or let's say, a number that you can't plan for) number of arguments to a function.

> *args is used to send a non-keyworded variable length argument list to the function.

```python
def arg_example(first_arg, *argv): # argv stands for argument vector! Like C++! Check the C++ tutorial for more examples!
    print("First arg:", first_arg)
    for arg in argv:
        print("Another arg through *argv:", arg)

arg_example('methylDragon','makes','orchestral','music')

''' Output:
First arg: methylDragon
Another arg through *argv: makes
Another arg through *argv: orchestral
Another arg through *argv: music
'''
```

**\*\*kwargs**

So we've seen how *args is used to pass a non-keyworded variable length list of arguments. **kwargs does this, but for named/keyworded arguments!

```python
def rawr(**kwargs):
    if kwargs is not None:
        for key, value in kwargs.items():
            print("%s says %s" % (key, value))

rawr(methylDragon="rawr", Rakkarn="sup")
# Output: methylDragon says rawr
# Rakkarn says sup
```

**Passing args and kwargs**

```python
# Let's say you want to write a list instead
# And you want to pass the list elements to the function
def test_function(p1, p2, p3):
    print(p1,p2,p3)
    return

# You can't pass a list or a tuple to this because the whole list or tuple will get passed as the first argument...
# SO HOW DO YOU DO IT? Args, duh.
args = (1,2,3)
test_function(*args)
# Now it'll print 1 2 3

# You can sort of do the same thing with kwargs also
kwargs = {"p1": 1, "p2": 2, "p3": 3} 
# Note, writing "p4": 4 will throw an exception since p4 was not defined in the function prototype
# Now it'll print 1 2 3
```

**Iterating over arguments passed by args**

```python
# If the function is defined 
def function_name(*args):
	# Then
    arguments = args
	# Will have arguments return a tuple of arguments
    
# If the function is defined
def function_name(**kwargs):
    # Then
    arguments = kwargs
    # Will have arguments return a dictionary of arguments
    
# You can then use these to iterate using for loops
```

**Args, Kwarg, Formal Arguments ordering**

```python
# The order is done in this way
function_name(arguments, *args, **kwargs)
```



### 2.5 Classes <a name="2.5"></a>

[go to top](#top)



### 2.6 Child Class Declaration <a name="2.6"></a>

[go to top](#top)



### 2.7 Virtual Methods and Polymorphisms <a name="2.7"></a>

[go to top](#top)



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

