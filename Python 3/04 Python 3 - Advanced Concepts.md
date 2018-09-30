# Advanced Python 3 Crash Course

Author: methylDragon  
Contains an advanced syntax reference for Python 3  
This time, we'll be going through many many (mostly unrelated) Python 3 coding concepts!    

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
   2.5   [Decorators](#2.5)    
   2.6   [Memoisation](#2.6)    
   2.7   [Copy](#2.7)    
   2.8   [Global, Local, and Nonlocal](#2.8)    
3. [Cool Stuff](#3)    




## 1. Introduction <a name="1"></a>

Ok. We've gone through all the basic stuff and simple syntax.

Now let's dive into the deep end. Very little hand holding here, but lots of useful concepts, and some pretty powerful stuff!  Also, since it can get pretty deep, I might make references to links to longer resources, and only mention the gist of what needs to be written so you can understand the concepts.

> I can't find a way to organise these, so just look through what looks interesting. I'll try to write pre-requisite concepts so you can have things to Google or refer to, but we're going to go pretty fast.

## 2. Advanced Python 3 Syntax and Concepts <a name="2"></a>

### 2.1 if \_\_name__ == "\_\_main__": <a name="2.1"></a>

[go to top](#top)

Looks intimidating at first, but it actually means something really simple.

Each time Python 3 opens and reads code, it sets several special (global) variables before executing said code. One of these is the `__name__` variable.

If the Python 3 main interpreter is running the program, (i.e. that the program is being run directly instead of through an import statement), `__name__` is set to `"__main__"`

```python
if __name__ == "__main__": # There we are!
    print("We're running the program directly!")
else:
    print("We're running the program indirectly! It's imported!")
```



### 2.2 lambda, map(), filter(), reduce() <a name="2.2"></a>

[go to top](#top)

map(), filter(), and reduce() operate on lists. But if we want to use them properly, we're going to have to go though lambda expressions first.

#### **lambda**

Lambda expressions are a good way to create throwaway functions! Just take note that they can't feature return statements, as their output is their default return output.

```python
# Observe the syntax
add_nums = lambda x, y: x + y

add_nums(1, 2) # Returns 3
add_nums(4, 5) # Returns 9
```

As you can see, this lambda expression for add_nums is more or less a function definition, turning add_nums into the name for a function.

Now we'll see how they can be used to create arbitrary anonymous functions

```python
# Define a function creates an anomymous function, and returns the output
def multiply_by(n):
    return lambda x : x * n

multiply_eight = multiply_by(8) # The function that is created multiplies the input by 8
multiply_six = multiply_by(6) # The function that is created multiplies the input by 6

multiply_eight(2) # Returns 16 (i.e. 2 * 8)
multiply_six(2) # Returns 12 (i.e. 2 * 6)
```

And even pair them with recursive functions!

```python
def makeTriangle(sign):
    def triangle(n):
        if n == 1:
            return sign+'\n'
        else:
            return triangle(n-1)+sign*n+'\n'
    return lambda x: triangle(x)

# Now you can do this!
print(makeTriangle("*")(5))

# Output:
# *
# **
# ***
# ****
# *****
```

However, the TRUE utility of lambda expressions comes when you pair them with map, filter, reduce!

#### **map()**

Map takes a list of items, and applies a function to every item in said list.

```python
# Let's do a map to square every number in a list!
my_nums = [1, 2, 3, 4, 5]

# Notice the lambda! We don't need to make a new named function for this!
map(lamda x: x ** 2, my_nums) # This creates a map object
my_squares = list(map(lamda x: x ** 2, my_nums)) # We need to create a list for it!

print(my_squares) # [1, 4, 9, 16, 25] We did it!

# Let's try one more, for uppercasing everything in an input string
rawr = "rawr"

RAWR = list(map(lambda x: x.upper(), rawr))

print(RAWR) # ['R', 'A', 'W', 'R']
```

#### **filter()**

Filter runs a function through every item in a list, creating a new list of items that fulfill the conditions set by said function.

More specifically items get added to the new list if the function that gets applied returns True

```python
# Let's filter out numbers that are not greater than 3
my_nums = [1, 2, 3, 4, 5]

print(list(filter(lambda x: x > 3, my_nums))) # [4, 5]
```

**reduce()**

Ok! We've done map and filter! **Reduce has to be imported.**

Reduce 'combines' items in a list, by running a function through every item in a list. 

The function that you use to run through the list has to take in two values, the first being the accumulated value, and the second being the current item you're iterating through in the list.

```python
from functools import reduce

my_nums = [1, 2, 3, 4]

# Remember, the first value is the accumulated value, and the second is the current value

# This one adds all values!
print(reduce(lambda x, y: x + y, my_nums)) # 10
# This one multiplies all values!
print(reduce(lambda x, y: x * y, my_nums)) # 24
# This one concatenates strings!
print(reduce(lambda x, y: x + y, ["methyl", "Dragon"])) # "methylDragon"
```



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
exponent_list = [x ** 2 for x in range(10)]
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

# This one uses two list comprehensions together
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

# Course, you can just use lambda functions to get the same result
# But it's less comprehensible (that was not meant to be a pun)
stuff = map(lambda w: [w.upper(), w.lower(), len(w)], words)
for i in stuff:
    print i
```

> **PS: These can be used as generators or in dictionaries (dictionary comprehensions!)** :D

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

#### **\*args**

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

#### **\*\*kwargs**

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

#### **Passing args and kwargs**

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

#### **Args, Kwarg, Formal Arguments ordering**

```python
# The order is done in this way
function_name(arguments, *args, **kwargs)
```



### 2.5 Decorators <a name="2.5"></a>

[go to top](#top)

If I don't explain it well enough for you, try: https://realpython.com/blog/python/primer-on-python-decorators/

Or a video tutorial: https://www.youtube.com/watch?v=FsAPt_9Bf3U

If you're having problems with docstrings also, do look up that link. They mention @wraps

#### **Pre-amble**

So you thought you knew functions. And sure you do!

You know how to:

```python
# Define them
def my_function(parameters):
    # Do stuff
    yield # Stuff
    return # Stuff also

# Compose them
def my_function(parameters):
    composed_function(other_parameters) # Call another function within a function!
    # Stuff
    return # Stuff

# Pass functions (and lambdas!) as parameters
# You can do this because in python, functions are first-class, i.e. they're objects that can be
# passed
def my_function(some_function):
    # Do stuff

my_function(lambda x : blahblah)
my_function(another_function)

# Return arbitrary ones
def my_function(parameters):
    # Do stuff
    return lambda x: #Some output
```

#### **Functions within Functions**

So it turns out, you can **define functions within other functions!**

```python
# Sorta like returning arbitrary ones with lambda expressions!
def outer_function():
    def inner_function()
        return
    return
```

#### **Decorators**

Decorators are functions that takes another function as an argument, and adds functionality to that function without changing its source code of the passed function. (Adding functionality to input functions!)

```python
# Simplest example
# Intuitively, decorators 'wrap' the extra functionality around our oiginal function

# before_after will be our decorator here:
def before_after(f): # Where f is a function
    def wrapper():
        print("before")
        f()
        print("after")
    return wrapper

# In effect, calling before_after(f)
# Is the same as calling
# print("before")
# f()
# print("after")
```
```python
# Now let's try it with a bunch of functions!
def hello():
    print("hello")

def rawr():
    print("rawr")

# You decorate a function by reassigning the function name to a function call of the decorator!
# All without changing the actual source code of rawr!
# This allows us to add extra functionality to functions that might have been messy to write in the
# function's code!
rawr = before_after(rawr) # Notice how it isn't rawr() !
rawr()

# Now rawr() runs the wrapper function, with f() being rawr()!
# Prints:
# before
# rawr
# after

# You can also apply the same decorator across different functions very easily!
hello = before_after(hello)
hello()

# Prints:
# before
# hello
# after
```

> **An important note!**
>
> ```python
> def before_after(f): # Where f is a function
>     def wrapper():
>         print("before")
>         f()
>         print("after")
>     return wrapper
> 
> # IS NOT
> 
> def before_after_wrong(f): # Where f is a function
>     print("before")
>     f()
>     print("after")
>     return f
> 
> # Here's the illustration
> 
> # We know how this goes
> # It changes rawr() into a decorator() call with rawr() as f()
> rawr = before_after(rawr)
> 
> # But
> # This won't actually change rawr()'s behaviour! Since it returns f, not decorator!
> rawr = before_after_wrong(rawr) # This will actually immediately do the decorator prints
> # As before_after_wrong(rawr) returns rawr
> # then: rawr = before_after_wrong(rawr)
> # is: rawr = rawr (but with the extra print statements as an extra effect)
> 
> rawr() # Behaves the same as rawr() before the 'decoration'!
> ```
>
> 

#### **Syntactic Sugar** (@ notation)

```python
# We've seen that we can apply a decorator by writing it as a composite function
rawr = before_after(rawr)

# There's a nicer way to do it though! Just use @ (it's called pie syntax)
# .. Yes. Seriously.

@before_after
def rawr():
    print("rawr")
    
@before_after
def hello():
    print("hello")
    
# Now you can do it across any number of functon definitions!
# You can even chain different decorators! It makes things really clear!

@decorator1 # Called last (outermost)
@decorator2 # Called second
@decorator3 # Called first (innermost)
def my_function:
    # blahblah
    
# Is equivalent to the less clear
my_function = decorator1(decorator2(decorator3(my_function)))
```

#### **Decorators with arguments**

Now we have a few problems. What happens when arguments are involved? This problem arises in two specific cases:

1. The original function had arguments
2. You want arguments to change how the decorator behaves

We can fix them easily!

```python
# Original function had arguments

# Just use *args and **kwargs! Now you can apply it to any given input function!
def decorator_function(f):
    def wrapper_function(*args, **kwargs): # Put the arguments on your wrapper!
        print("decorator stuff")
        return f(*args, **kwargs)
    return wrapper_function

# Let's observe what happens with an example!

@decorator_function
def my_function(num1, num2):
    print(num1)
    print(num2)
    return num1 + num2

print(my_function(1,5))
# Output
# decorator stuff
# 1
# 5
# 6
```

```python
# Arguments change how the decorator behaves
# Note: This one gets a little complicated...

# Put your arguments in, no fear! But have an extra layer of wrapping!
def decorator_function(arguments): # Put your arguments on your decorator function!
    def outer_wrapper(f): # The outer_wrapper takes care of the input function!
        def wrapper_function():
            return f() # Manipulate your f with the arguments here!
        return wrapper_function
    return outer_wrapper

# An example will help a lot here!

def multiply_by(num): # A call to this decorator
    def function_wrapper(f):
        def wrapper_function(*args):
            return f(*args) * num # Because we know our f() will take arguments
        return wrapper_function 
        # Returns a function that multiplies the original input function's output by num
    return function_wrapper
    # And adds that functionality to the input function

# This is equivalent to

# First making a decorator
# Let's say we make num = 3
def multiply_by_three(f):
    def wrapper():
        return f() * 3
    return wrapper

# Then applying the decorator
@multiply_by_three
def decorated_function():
    return # blah blah

# Let's test it out!
@multiply_by(3)
def multiply_by_3(num):
    return num

multiply_by_3(3) # Returns 9 
```

#### **Practical Examples**

```python
# Source: https://github.com/CoreyMSchafer/code_snippets/blob/master/Decorators/snippets.txt

# Decorators, implemented using a class
class decorator_class(object):

    def __init__(self, original_function):
        self.original_function = original_function

    def __call__(self, *args, **kwargs):
        print('call method before {}'.format(self.original_function.__name__))
        self.original_function(*args, **kwargs)

# Logging decorator
def my_logger(orig_func):
    import logging
    logging.basicConfig(filename='{}.log'.format(orig_func.__name__), level=logging.INFO)

    def wrapper(*args, **kwargs):
        logging.info(
            'Ran with args: {}, and kwargs: {}'.format(args, kwargs))
        return orig_func(*args, **kwargs)

    return wrapper

# Timer decorator
def my_timer(orig_func):
    import time

    def wrapper(*args, **kwargs):
        t1 = time.time()
        result = orig_func(*args, **kwargs)
        t2 = time.time() - t1
        print('{} ran in: {} sec'.format(orig_func.__name__, t2))
        return result

    return wrapper
```

Hopefully these toy examples help you out! I'll be using decorators in some of the other sections too! So it wouldn't hurt to learn it!

But, if you want to go deeper, with classes, for example: https://www.codementor.io/sheena/advanced-use-python-decorators-class-function-du107nxsv



### 2.6 Memoisation <a name="2.6"></a>

[go to top](#top)

**Pre-Requisites**

- Recursive functions
- Decorators

#### **Introduction**

Memoisation is a way to speed up computation time for recursive functions, by ensuring that repeat computations do not happen! (It can potentially reduce O(n^2) complexity algorithms to O(n)! It's very important in dynamic programming problems!)

Consider this recursive function, one that calculates the Fibonacci numbers up to the input number

```python
def fib(num):
    if num == 0:
        return 0
    elif num == 1:
        return 1
    else:
        return fib(num - 1) + fib(num - 2)
```

One can easily see that at high values of num, there will be many repeat calls of lower numbers!

```python
# For example

fib(5)

# Will result in a call to:
# fib(4) + fib(3)
# Which splits into:
# fib(3) + fib(2) + fib(2) + fib(1)
# fib(2) + fib(1) + fib(2) + fib(2) + fib(1)

# Already there's three repeat calls to fib(2)! Now, if only there was a way to store the answer
# And substitute it in instead of splitting the calls again...
```

#### **Implementation**

Let's see how we can do this! It's very easy!

```python
# Create a memoisation decorator
def memoise(f):
    memo = {}
    def checker(x):
        if x not in memo:            
            memo[x] = f(x)
        return memo[x]
    return checker

# And apply it! Done!
@memoise
def fib(num):
    if num == 0:
        return 0
    elif num == 1:
        return 1
    else:
        return fib(num - 1) + fib(num - 2)
```

```python
# https://www.python-course.eu/python3_memoization.php
# mentions a way to do it with a class as well

class Memoize:
    def __init__(self, fn):
        self.fn = fn
        self.memo = {}
    def __call__(self, *args):
        if args not in self.memo:
        self.memo[args] = self.fn(*args)
        return self.memo[args]

@Memoize
def fib(n):
    if n == 0:
        return 0
    elif n == 1:
        return 1
    else:
        return fib(n-1) + fib(n-2)
```



### 2.7 Copy <a name="2.7"></a>

[go to top](#top)

(Copy and Deepcopy behave the same if the objects being copied are immutable types, so what I'll write here only applies otherwise.)

Use the copy module if you're having trouble recursively copying lists or other mutable types!

```python
import copy

# With assignment, you don't create a new object
# b_list is simply an alias for a_list
a_list = [5] #Some Mutable Type
b_list = a_list

# Shallow copying, copies references to nested mutable objects,
# c_list is a new object
# But contains references to the objects in a_list
c_list = copy.copy(a_list)
print(id(c_list) is id(a_list) # False
print(id(c_list[0]) is id(a_list[0])) # True

# Deep copying, copies the nested objects instead of references
# d_list is a new object
# And contains objects that are the same value as, but are distinct from those in a_list
d_list = copy.deepcopy(a_list)
print(id(d_list) is id(a_list)) # False
print(id(d_list[0]) is id(a_list[0])) # False
```



### 2.8 Global, Local, and Nonlocal <a name="2.8"></a>

Let's just get a refresher on variable scopes first

#### **Local variables**

```python
x = "outer"

def inner():
    # Notice how the reassignment of this x doesn't change the outer x
    # That is the hallmark of a local variable!
    x = "inner"
    
    return x

print("outer:", x)
print("inner:", inner())

'''
Output:
outer: outer
inner: inner
'''
```

As you can see, variables declared within a function (and other kinds of scopes, like classes, and class methods), are **local** to their scope. In other words, they don't interact with anything outside of the scope!

Here's one more example

#### **Local variables (one level deeper!)**

```python
x = "outer"

def inner():
    x = "inner"
    
    def even_inner():
        # x is local to inner(), which encompasses even_inner() so this works
        print("even_inner:", x) 
        
        # x = 10 <-- This will break it though, since x becomes local to even_inner
        # Which will cause the previous statement to break as inner's x gets deassigned
        # within even_inner
        
        return x # Notice that this still works!
    
    return even_inner()

print("outer:", x)
print("inner:", inner())

'''
Output:
outer: outer
even_inner: inner
inner: inner
'''
```

#### **global and nonlocal**

Ok. That was troublesome.

Did you know that there is a way to control what scope variables fall into? This way you can have variables that would have been local affect the global variable (or a parent scope's variable) instead!

Use the keywords `global` and `nonlocal`!

#### **global**

```python
x = "unaffected"

def inner():
    # This sets any assignments to the name 'x' to be the same as assignments
    # to the global x
    global x
    
    x = "affected"
    
    
print("before inner:", x)
inner()
print("after inner:", x)

'''
Output: (We can see this works!)
before inner: unaffected
after inner: affected
'''
```

#### **nonlocal**

Non-local is something similar, but different.

Where `global` allows you to assign to variables in the global scope.

`nonlocal` instead allows you to go 'one-step up only'. Allowing you to change local variables local to a **parent** scope instead of the global scope! It's a good way of encapsulating variables and preventing stuff from 'spilling out', so to speak.

It's best illustrated with an example:

```python
x = "global"

def inner():
    x = "inner" # This sets up a variable local to inner
    print("before even_inner:", x) # We verify it here
    
    def even_inner():
        nonlocal x # Now we can reassign the x that is local to inner!
        x = "even_inner" # Which we do here
    
    even_inner() # This sets "inner" -> "even_inner"
    
    return x # And returns "even_inner"
    
print("global:", x)
print("after even_inner:", inner())
print("global preserved?:", x)

'''
Output:
global: global
before even_inner: inner
after even_inner: even_inner
global preserved?: global
'''
```

Now you have the power!



## 3. Cool Stuff <a name="3"></a>

\# Cool Stuff

#### Remove Overhead

- Throw a decorator on stuff, remove Python overhead (where applicable!): 
  - http://numba.pydata.org/
- Write some parts of your code in C, and throw it in a different interpreter!
  - http://cython.org/



#### ML

- http://scikit-learn.org/ !!!
- http://pytorch.org/
- https://www.tensorflow.org/
- https://opencv.org/


#### Making Things Look Pretty

- Colour your console output!
  - https://pypi.python.org/pypi/colorama




```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

​    

------

 [![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)