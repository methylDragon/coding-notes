# C++ Headers

Author: methylDragon  
Header files!   
Information source: http://www.cplusplus.com/articles/Gw6AC542/

------

## Pre-Requisites

### Assumed knowledge (This is a C++ crash course, not a basic coding tutorial)

- How **variables, loops, conditionals, etc**. work (Basic coding fundamentals will help a lot!)
- Linux (**Terminal/Console proficiency**) (We're going to need to compile our stuff)
- Gone through the all preceding parts of the tutorial



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Header Files](#2)    
   2.1   [Introduction](#2.1)    
   2.2   [Header Files](#2.2)    
   2.3   [Header Guards](#2.3)    
   2.4   [Tips on #Including](#2.4)    
   2.5   [Inline Functions](#2.5)    



## 1. Introduction <a name="1"></a>

So sometimes you see .hpp or .h files appearing instead of .cpp in other people's source code. Those are header files! Ever wondered what they were for?

>If you're just starting out in C++, you might be wondering why you need to #include files and why you would want to have multiple .cpp files for a program. The reasons for this are simple:
>
>**(1)** It speeds up compile time. As your program grows, so does your code, and if everything is in a single file, then everything must be fully recompiled every time you make any little change. This might not seem like a big deal for small programs (and it isn't), but when you have a project of reasonable size, compile times can take *several minutes* to compile the entire program. Can you imagine having to wait that long between every minor change?
>
>Compile / wait 8 minutes / "oh crap, forgot a semicolon" / compile / wait 8 minutes / debug / compile / wait 8 minutes / etc
>
>**(2)** It keeps your code more organized. If you seperate concepts into specific files, it's easier to find the code you are looking for when you want to make modifications (or just look at it to remember how to use it and/or how it works).
>
>**(3)** It allows you to separate *interface* from *implementation*. If you don't understand what that means, don't worry, we'll see it in action throughout this article.
>
>Source: http://www.cplusplus.com/articles/Gw6AC542/ (Why we need header files)

The whole point of header files is to continue the tradition of avoiding as much instances of needing to copy-paste code as possible, since that leaves the door open for bugs and a much harsher time spent maintaining code.

Bottom line is, you'll start seeing header files get more and more useful once you move beyond writing just a single .cpp file, and start writing multiple source files that talk to each other (which is far more efficient anyway...)



## 2. Header Files <a name="2"></a>

### 2.1 Introduction <a name="2.1"></a>

[go to top](#top)

So the thing about C++ is that it's a bit finicky at times because of how programs are compiled. What happens is each individual source file (.cpp file) is compiled alone, and THEN linked together by the compiler to form the final binary.

This is problematic because the individual .cpp files don't know what's going on in the other .cpp files, so you can get import or declaration errors!

Let's look at this example from http://www.cplusplus.com/articles/Gw6AC542/

```c++
// in myclass.cpp

class MyClass
{
public:
  void foo();
  int bar;
};

void MyClass::foo()
{
  // do stuff
}
```

```c++
// in main.cpp

int main()
{
  MyClass a; // Compiler error: 'MyClass' is unidentified
  return 0;
}
```

Since MyClass was not defined in main.cpp, you'll get a compiler error even though you made provisions for it in myclass.cpp!

How do we get around that? Funny you'd ask that...



### 2.2 Header Files <a name="2.2"></a>

[go to top](#top)

Header files are used to make an **interface** separate from implementation. Think OOP PHP interfaces (check my tutorial for that.) This allows you to speed up compile time because you can just resettle the interface instead of redoing every source file.

Again, the header files are the **interface** which allow other source files to **interact with the implementation** defined in the source (.cpp) files that are associated with their relevant header files.

Here's the example from: http://www.cplusplus.com/articles/Gw6AC542/

**The Header File Definition**

```c++
// in myclass.h

class MyClass
{
public:
  void foo();
  int bar;
};
```

You can now include it in the other source files using #include "myclass.h"! (#include works like a copy paste operation, whatever you wrote in myclass.h ends up getting copy-pasted in the spot.)

Notice how the header file doesn't actually say what the data types or methods store, but just the names and types. This is how it is an interface, it's a box for the individual source files to fill.

As such

```c++
// in myclass.cpp
#include "myclass.h"

void MyClass::foo()
{
    // some implementation
}
```

```c++
//in main.cpp
#include "myclass.h"  // defines MyClass

int main()
{
  MyClass a; // no longer produces an error, because MyClass is defined
  return 0;
}
```

Remember! Each individual .cpp is compiled independently, and then linked together. Including the header file in the main.cpp source file lets it play with the interface (so you've declared it and it's aware of it), but leaves the implementation to be done by the myclass.cpp source file, which will eventually get linked to main.cpp once the entire program is compiled.



### 2.3 Header Guards <a name="2.3"></a>

[go to top ](#top)

Course, you might run into problems if you try to included stuff that's already been included. So use header guards!

I already went through this in section 05 of this tutorial - Tips and Tricks

Here's a refresher. Remember to use #define !

```c++
#ifdef myIdentifier
// Compile this if myIdentifier is defined
// Put your #define and then the #include here!
#else
// Otherwise compile this
#endif // Otherwise it'll just skip it
```





### 2.4 Tips on #Including <a name="2.4"></a>

[go to top](#top)

This section I'm just taking wholesale from http://www.cplusplus.com/articles/Gw6AC542/ because it's handy enough...

> There are two basic kinds of dependencies you need to be aware of:
> 1) stuff that can be forward declared
> 2) stuff that needs to be #included
>
> If, for example, class A uses class B, then class B is one of class A's dependencies. Whether it can be forward declared or needs to be included depends on how B is used within A:
>
> - **do nothing if:** A makes no references at all to B
> - **do nothing if:** The only reference to B is in a friend declaration
> - **forward declare B if:** A contains a B pointer or reference: B* myb;
> - **forward declare B if:** one or more functions has a B object/pointer/referenceas a parementer, or as a return type: B MyFunction(B myb);
> - **\#include "b.h" if:** B is a parent class of A
> - **\#include "b.h" if:** A contains a B object: B myb;`
>
> You want to do the least drastic option possible. Do nothing if you can, but if you can't, forward declare if you can. But if it's necessary, then #include the other header.
>
> Ideally, the dependencies for the class should be layed out in the header. Here is a typical outline of how a "right way" header might look:
>
> **myclass.h**
>
> ```c++
> //=================================
> // include guard
> #ifndef __MYCLASS_H_INCLUDED__
> #define __MYCLASS_H_INCLUDED__
>
> //=================================
> // forward declared dependencies
> class Foo;
> class Bar;
>
> //=================================
> // included dependencies
> #include <vector>
> #include "parent.h"
>
> //=================================
> // the actual class
> class MyClass : public Parent  // Parent object, so #include "parent.h"
> {
> public:
>     std::vector<int> avector;    // vector object, so #include <vector>
>     Foo* foo;                    // Foo pointer, so forward declare Foo
>     void Func(Bar& bar);         // Bar reference, so forward declare Bar
>
>     friend class MyFriend;       // friend declaration is not a dependency
>                                   //   don't do anything about MyFriend
> };
>
> #endif // __MYCLASS_H_INCLUDED__ 
> ```
>
> This shows the two different kinds of dependencies and how to handle them. Because MyClass only uses a pointer to Foo and not a full Foo object, we can forward declare Foo, and don't need to #include "foo.h". *You should always forward declare what you can -- don't #include unless it's necessary*. Unnecessary #includes can lead to trouble.
>
> If you stick to this system, you will bulletproof yourself, and will minimize #include related hazards.

> The "right way" I illustrated above is all about encapsulation. Files that want to use MyClass don't need to be aware of what MyClass uses in order for it to work, and don't need to #include any MyClass dependencies. All you need to do to get MyClass to work is #include "myclass.h". Period. The header file is set up to be completely self contained. It's all very OO friendly, very easy to use, and very easy to maintain.
>
> Source: http://www.cplusplus.com/articles/Gw6AC542/

### 2.5 Inline Functions <a name="2.5"></a> 

[go to top](#top)

Next problem! If you have functions defined in source files that are meant to be used in other source files, you're going to have to copy paste the same function definition into every file that uses them!

So it turns out you can just put function definitions inside your header as well since the #include of the header means the definitions get copied in. You do this by specifying the `inline` keyword, which tells C++ to use the definition wherever its called!

And you do this by defining the functions OUTSIDE of any class definitions that you specified in the header file!

Example:

```c++
// b.h  (assume its guarded)

//------------------
class A;  // forward declared dependency

//------------------
class B
{
public:
  void Func(const A& a);  // okay, A is forward declared
};

//------------------
#include "a.h"        // A is now an include dependency

inline void B::Func(const A& a)
{
  a.DoSomething();    // okay!  a.h has been included
}
```

**Or if you don't like seeing function definitions at the end of a header file**, just write ANOTHER header file and stick it there!

```c++
// b.h

    // blah blah

class B { /* blah blah */ };

#include "b_inline.h"  // or I sometimes use "b.hpp" 
```

```c++
// b_inline.h (or b.hpp -- whatever)

#include "a.h"
#include "b.h"  // not necessary, but harmless
                //  you can do this to make this "feel" like a source
                //  file, even though it isn't

inline void B::Func(const A& a)
{
  a.DoSomething();
}
```



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

​    

------

 [![Yeah! Buy the DRAGON a COFFEE!](../../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)