# Advanced C++ Crash Course (Pointers)

Author: methylDragon  
Contains an advanced syntax reference for C++  
This time, we'll be going through Pointers and Memory Management!   
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=Rub-JsjMhWY

------

## Pre-Requisites

**Assumed knowledge (This is a C++ crash course, not a basic coding tutorial)**

- How **variables, loops, conditionals, etc**. work (Basic coding fundamentals will help a lot!)
- Linux (**Terminal/Console proficiency**) (We're going to need to compile our stuff)
- Gone through the all preceding parts of the tutorial



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [C++ Pointer Syntax Reference](#2)    
   2.1   [Memory Addresses](#2.1)    
   2.2   [Passing by Value vs Passing by Reference](#2.2)    
   2.3   [References](#2.3)    
   2.4   [Pointers](#2.4)    
   2.5   [Arrays with Pointers](#2.5)    
   2.6   [Indirection (Pointers of Pointers)](#2.6)    
   2.7   [When to use Pointers and when to use References](#2.7)    
   2.8   [Memory Management](#2.8)    



## 1. Introduction <a name="1"></a>

Ok. So we've covered the basics of C++ that overlap with most other programming languages. Now we'll dive into the deep end of what makes C++, well, C++. Memory management, pointers, and the like.



## 2. C++ Pointer Syntax Reference <a name="2"></a>

### 2.1 Memory Addresses <a name="2.1"></a>

[go to top](#top)

https://gist.github.com/ericandrewlewis/720c374c29bbafadedc9

Basic understanding: Whenever you create a variable, say...

```c++
int myNumber = 8;
```

The variable, myNumber is another name for a **memory address** that **stores** the value 8. Every variable does this. (Example memory address: 0x1001054a0)

>  Imagine that the memory address is a box that holds the value! The variable name then is a user-friendly way to give the box a name to refer to it (instead of having to type out the memory address explicitly!)

We use these variable names to manipulate the values stored within their respective **memory addresses**.

In order to print out the memory address of a variable, use the **reference** ('address-of') operator (&)

```c++
std::cout << "The variable's value lives at memory address " << &myVariable << "\n"; // This prints out the memory address of myVariable
```



### 2.2 Passing by Value vs Passing by Reference <a name="2.2"></a>

[go to top](#top)

https://www.reddit.com/r/explainlikeimfive/comments/2gl0xh/eli5programming_value_vs_reference/

This leads us to a very **important** concept in C and C++. The difference between passing by **value** and passing by **reference**.

Let's illustrate the important difference between the two using an example.

```c++
void myMultiplicationFunction(int x) 
{
  x = x * 2;
}

int main()
{
	int x = 2;
	myMultiplicationFunction(x);
  	std::cout << "Multiplied x is " << myMultiplicationFunction(x) << '\n';
	std::cout << "x is " << x;
}

/* OUTPUT:
Multiplied x is 4
x is 2
*/
```

This is **very weird**. Didn't the function manipulate x, since we passed it?

Well, actually, not really. The key understanding is that there are **TWO** ways to pass variables to functions.

> Passing by **Value**: You pass a COPY of the value stored inside a variable
>
> - This means the original value stored in the variable **DOESN'T CHANGE**

> Passing by **Reference**: You pass the variable itself! (By passing the reference TO the variable)
>
> - This is the **only way to change** the value stored in a variable!

The default way C++ passes variables to functions when you write them as arguments is through passing by **value**!

So **how** do we pass by reference? You can do it in two ways. Either:

>a) Access the value via **pointers**
>
>b) Set the argument to be the variable itself via **references**

The concepts needed to understand them will be explained in the next section

```c++
// Via pointers (a)
void myMultiplicationFunction(int *x) // Pass this function addresses (&parameter)
{
  *x = x * 2;
}

// Example call: 
// int a = 2;
// myMultiplicationFunction(&a);

// Via references (b)
void myMultiplicationFunction(int &x) // Pass this function variables! (parameter) it'll find the address itself!
{
  x = x * 2;
}

// Example call: 
// int a = 2;
// myMultiplicationFunction(a);
```



### 2.3 References <a name="2.3"></a>

[go to top](#top)

**Recall:** Every variable that you define is another name (an **alias**) of a memory address. Imagine these memory addresses as boxes to store values in, and the variable names as a way to refer to the box.

You can create MORE **aliases** for the variables! These are called **references**.

```c++
// Let's say we have a variable defined
int i = 5;

// To define an alias (r) for it, we use
int &r = i; // Now, manipulating r, does the same thing as manipulating i
// Note: int& r = i; will ALSO work!

// What we're essentially saying, is, let the address of r (&r)  be the same as the address defined by i. So now r and i refer to the SAME memory address.
// r does NOT STORE the memory address, r BECOMES another name for the memory address that i is the alias for
```

To illustrate what we've just done, examine the output of the following program:

```c++
cout << "r = " << r << endl;
cout << "i = " << i << endl;
r++;
cout << "i = " << i << endl;

/* OUTPUT:
r = 5
i = 5
i = 6
*/

// Notice how i changed even though we didn't explicitly manipulate i, but r instead? r is indeed an alias for i!
```

**Note that there are some restrictions on references though!:**

> - References cannot be NULL (they must always be an alias for something that exists)
> - References cannot be changed to be an alias for another object/variable
> - A reference must be initialised when it is created



### 2.4 Pointers <a name="2.4"></a>

[go to top](#top)

Pointers are **variables** in and of themselves, that **STORE MEMORY ADDRESSES**. In effect, they act as signposts that POINT to memory addresses!

> A pointer `x` that points to variable `y` will have its own memory address, but will store the memory address that y is the alias for.

Pointers are the more versatile, but less 'safe/secure' cousin to references.

```c++
// You define a pointer by using the * operator
int myNumber = 5;
int* myPointer = &myNumber; // Now myPointer stores the memory address of myNumber, which is an int
// Note: int *myPointer = &myNumber; will ALSO work!
```

Now we can manipulate and access this variable! We access the data stored inside the memory address pointed to by the pointer using the **dereference** operator (*)

```c++
cout << "Address of pointer: " << myPointer << endl;
// To access the data stored inside the memory address pointed to by the pointer, use the asterisk again. But this time it's used as a 'dereferencing operator'
cout << "Number stored: " << *myPointer << endl;

/* OUTPUTS:
<some address>
5
*/
```

Final intuitions:

References are the ALTERNATE NAMES you give to memory addresses. When you increment a reference, you are incrementing the value contained within the memory address!

Pointers STORE the memory addresses. When you increment a pointer, you are incrementing the memory address (the label), not the value contained within it!

> (https://www.reddit.com/r/learnprogramming/comments/1w6v07/eli5_pointers_and_their_usefulness/)
>
> Pointers, ELI5 version:
>
> Let's say you want to remember part of a book for later.  What would you do?
>
> You could write the entire passage down on another piece of paper so that you can remember for later, but that takes a while and might use a lot of paper.
>
> Instead, you could just write down the page number and look for the passage later. That's a lot faster to write down and won't use as much paper.
>
> The second example uses a "pointer" to the passage you want to remember for later.
>
> However, the first example may be a bit more convenient, depending on the size of the passage.  You're also able to change the passage in the first example to whatever you want without ruining your book.
>
> ------
>
> This is a nice analogy because it's easy to extend:
>
> You want to look up a specific topic in a book so you open it and find the index. The index says the glossary is on page 200. In the glossary you find that the phrase "intermediate value theorem" is referenced on page 106, and 110. Pointers to pointers!



### 2.5 Arrays with Pointers <a name="2.5"></a>

[go to top](#top)

Basic concept: When an array is initialised in C++, each successive array element is actually in successive memory addresses! So when you increment a pointer that points to an array, you'll change the value that is pointed to to the next element!

```c++
// Let's try this out!
int numbers[5] = {1, 2, 3, 4, 5}; // Initialise an array
int* numbersPtr = numbers; // And a pointer which stores the pointer to the first element!

cout << "Before: " << *numbersPtr << endl;

numbersPtr++; // We increment the pointer here

cout << "After: " << *numbersPtr << endl;

/* OUTPUTS:
Before: 1
After: 2
*/
```

The variable name of an array is also just a pointer to the array!

```c++
// Let's try this out!
int numbers[5] = {1, 2, 3, 4, 5}; // Initialise an array

cout << "Before: " << *numbers << endl;

numbersPtr++; // We increment the pointer here

cout << "After: " << *numbers << endl;

/* OUTPUTS:
Before: 1
After: 2
*/
```

> So because of this. Just remember that array variables are special, since they're not really variables (aliases), but pointers that point to the array. So passing arrays to functions will result in you passing by reference!



### 2.6 Indirection (Pointers of Pointers) <a name="2.6"></a>

[go to top](#top)

Basically, pointers to pointers (ad nauseum)

If you think of a first-level pointer (say... int*) as an array, then a second-level pointer (int**) is an array of arrays! The cool thing about doing nth-dimensional arrays via pointers, is that the sizes don't have to be set in stone for the lowest level array!

```c++
// Here's an example

int numbers[5][5]; // This will give you 25 memory blocks worth of data. 5 arrays of size 5
int* numbers[5]; // This will give you n * 5 memory blocks worth of data, where n is the size of the particular array being pointed at

// You can do this ad nauseum, creating monstrosities like int************
```

Why is this useful? Let's ask stackoverflow!

https://stackoverflow.com/questions/5580761/why-use-double-pointer-or-why-use-pointers-to-pointers

> Here is a SIMPLE answer!!!!
>
> - lets say you have a pointer that its value is an address.
> - but now you want to change that address.
> - you could, by doing pointer1 = pointer2, and pointer1 would now have the address of pointer2.
> - BUT! if you want a function to do that for you, and you want the result to persist after the function is done, you need do some extra work, you need a new pointer3 just to point to pointer1, and pass pointer3 to the function.

Another example of indirection being used, is with strings!

> If you want to have a list of characters (a word), you can use `char *word`
>
> If you want a list of words (a sentence), you can use `char **sentence`
>
> If you want a list of sentences (a monologue), you can use `char ***monologue`
>
> If you want a list of monologues (a biography), you can use `char ****biography`
>
> If you want a list of biographies (a bio-library), you can use `char *****biolibrary`
>
> If you want a list of bio-libraries (a ??lol), you can use `char ******lol`



### 2.7 When to use Pointers and when to use References <a name="2.7"></a>

[go to top](#top)

General maxim: `Use reference wherever you can, pointers wherever you must.`

Use pointers if you don't want to initialise it at point of declaration. (Declaring an object means you create it. Initialise means you assign a value to it as well as creating it.) 

You HAVE to initialise a reference when you create it because a reference cannot be an alias for nothing/NULL, but pointers CAN!!

Pointers can also be altered to point to other memory addresses! But references are stuck with being an immutable alias for only ONE object!



### 2.8 Memory Management <a name="2.8"></a>

[go to top](#top)

Read more: http://www.cplusplus.com/doc/tutorial/dynamic/

I have to say it here, since we've covered pointers. The reason why C++ is so powerful (and hard), is because you have to control how memory is managed in the program. This means cleaning up after yourself if you want to no longer use variables, controlling the memory flow, assigning pointers, and all the like. It goes really, really deep.

So here's a small primer! For **dynamic (runtime) memory management.**



#### **new and new[]**

> You use `new` if you want objects to persist until you `delete` it. But if you don't specifically need that, DO NOT USE NEW because it's expensive memory-wise.

`new` reserves memory, and returns a pointer to the memory that was allocated! If you assign an array, then it returns the pointer to the first element of the array.

```c++
int * foo; // Initialise a pointer
int * bar; // Another one

foo = new int; // And allocate memory to it
bar = new int [5]; // Or perhaps an array of size 5?
```

>  But be careful! Memory allocated in this way needs to be disposed of manually using `delete.`
>
> Otherwise it becomes a memory leak, which, if accumulated enough such that your program runs out of the memory allocated to it, crashes.



#### **delete and delete[]**

`delete` is how you do garbage disposal. Be wary of using pointers you've deleted though! Since the memory space previously allocated to the pointer could have been used for other things, or would be filled with garbage.

```c++
delete ptr; // To delete pointers
delete[] array_ptr; // To delete array pointers
```



#### **Assignment Failures**

From: http://www.cplusplus.com/doc/tutorial/dynamic/

If you try to store a variable that will take too much memory than the memory that is available, the assignment statement will fail.

**You can catch this exception!**

Alternatively, if you want to still let it pass, then use `nothrow`, which will return a null pointer instead (which is still bad, and you still have to verify if it worked, so it's less efficient, but easier to write intially...)

```c++
 foo = new int [5];  // if allocation fails, an exception is thrown 

// Or using (nothrow)

int * bar;
bar = new (nothrow) int [5];
if (bar == nullptr) { // Check to see if bar was assigned a null pointer
  // error assigning memory. Take measures.
}
```



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```
​        

---

 [![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)