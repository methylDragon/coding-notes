# C++ Tips and Tricks

Author: methylDragon  
Let's go!! Pro-tip time!   
I'll be adapting it from The Happie Cat's "Top 10 Things I Wish I Had Known When Learning C++": https://www.youtube.com/watch?v=9zgAcSEmwNo (The original video got removed :( )

------

## Pre-Requisites

**Assumed knowledge (This is a C++ crash course, not a basic coding tutorial)**

- How **variables, loops, conditionals, etc**. work (Basic coding fundamentals will help a lot!)
- Linux (**Terminal/Console proficiency**) (We're going to need to compile our stuff)
- Gone through the all preceding parts of the tutorial



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Tips and Tricks](#2)    
   2.1   [C++ is NOT C](#2.1)    
   2.2   [C++ is just moving memory around](#2.2)    
   2.3   [Plan your code before your write it](#2.3)    
   2.4   [The Rubber Ducky Method](#2.4)    
   2.5   [Organise your code](#2.5)    
   2.6   [Pointer Arithmetic](#2.6)    
   2.7   [Use const and scoping](#2.7)    
   2.8   [Header Guards](#2.8)    
   2.9   [Try using a UNIX platform](#2.9)    
   2.10 [Effective C++](#2.10)    



## 1. Introduction <a name="1"></a>

Alright! Now let's deal with some issues that might pop up or random problems you might come across (that are due to C++'s quirks instead of your own user errors) and spend waaaaay too long solving!



## 2. Tips and Tricks <a name="2"></a>

### 2.1 C++ is NOT C <a name="2.1"></a>

[go to top](#top)

It might have been very closely related in the past, but they're more or less very different now. C++ has extra stuff like references, and virtual functions, whereas C does not.

Don't make yourself look dumb!



### 2.2 C++ is just moving memory around <a name="2.2"></a>

[go to top](#top)

If C++ is too hard to get your head around, go look up a tutorial on computer architecture! (Binary, to Assembly, opcodes, and the like.) If you want a game to play to learn it, I recommend **Human Resource Machine**.



### 2.3 Plan your code before you write it <a name="2.3"></a>

[go to top ](#top)

Go old school, with pen and paper. Draw your program-flow charts and what you intend the program to do BEFORE actually getting to writing it!

It'll help you debug faster, and help you actually make sure you implement everything you wanted to to begin with.



### 2.4 The Rubber Ducky Method <a name="2.4"></a>

[go to top](#top)

So it turns out there's a secret weapon in the programmer's arsenal for whenever they encounter a horrific bug.

Get a small toy, or a friend. AND EXPAIN YOUR CODE TO THEM. Or maybe explain your problem to them, framing it as a question. Sometimes flaws in your logic suddenly become obvious. Woo.



### 2.5 Organise your code <a name="2.5"></a> 

[go to top](#top)

- Classes should do ONE THING (Chessboards shouldn't have methods for every single chess-PIECE)
- Functions should be 25 lines OR LESS. Makes it easier to debug. If they're longer, split it up.
- COMMENT YOUR CODE. You and your peers will thank me in a couple years' time. It's very easy to forget what you wanted to do a couple of weeks or months after leaving or finishing a project. Comments help a lot.



### 2.6 Pointer Arithmetic <a name="2.6"></a>

[go to top](#top)

POP QUIZ. If you don't understand what's going on here... get a refresher, and check out the pointer section.

Pointers are arguably the most important part of C++!

```c++
int x = 3;
int* y = 4;
*y = 2;
y = &x;
```



### 2.7 Use const and scoping <a name="2.7"></a>

[go to top](#top)

Use them to protect your variables even if it seems like it doesn't matter!!

const is a keyword! Make variables that are supposed to be constant IMMUTABLE!

Example of scoping:

```c++
blah(){ // THE BRACKETS ARE THE SCOPE. They prevent naming conflicts by keeping them LOCAL
  	// stuff
}
```



### 2.8 Header Guards <a name="2.8"></a>

[go to top](#top)

> Beware the difference between #define and #include !
>
> \#define tells the compiler that you're defining something
>
> YOU MUST INCLUDE IT TO USE HEADER GUARDS!
>
> \#include includes the stuff... pretty self-explanatory

Code examples from: http://faculty.cs.niu.edu/~mcmahon/CS241/c241man/node90.html

`#ifdef` and `#ifndef` are conditional compilation macros used in C++

Basically, they allow you to ask C++ to ignore certain parts of code IF a particular name, or keyword has been, or has not been defined.

```c++
#ifdef myIdentifier
// Compile this if myIdentifier is defined
#else
// Otherwise compile this
#endif // Otherwise it'll just skip it
```

Why is this useful? Let's get the example from the link I pasted.

Two classes and their header files are shown below:

```c++
                           #include "A.h"          #include "A.h"
class A                    class B                 #include "B.h"
{                          {
  int x;                      A a;
public:                       int y;               A a;
};                         public:                 B b;
                           };


     A.h                       B.h                      htest.cc
```

```c++
// Running it gives errors!

/* ERRORS:
In file included from B.h:1,
                 from htest.cc:2:
A.h:2: error: redefinition of `class A'
A.h:2: error: previous definition of `class A'
*/

// Because #include "A.h" was defined twice
```

We could either solve this problem by manually removing the double inclusion, but that might break certain file relations (for example, if B is no longer initialised, then htest.cc will throw up an error because the identifier for class A didn't get defined.)

You can see here then, that conditional compilation using the header guards `#ifndef` and `#enddef` will be perfect for this, more or less! That way, you can include the `#include` lines but without having to worry about repeats!

Using the example above, the files become:

```c++
#ifndef A_H                #ifndef B_H
#define A_H                #define B_H 

                           #include "A.h"          #include "A.h"
class A                    class B                 #include "B.h"
{                          {
  int x;                      A a;
public:                       int y;               A a;
};                         public:                 B b;
                           };

#endif /* A_H */           #endif /* B_H */

     A.h                       B.h                      htest.cc
```



### 2.9 Try using a UNIX platform <a name="2.9"></a>

[go to top](#top)

Instead of using fancy tools! Start from a simpler tool first to get better acquainted with C++



### 2.10 Effective C++ <a name="2.10"></a>

[go to top](#top)

^^^^^^^^^^^^^^^^^^ This book is nice.



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

​        

---

 [![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)