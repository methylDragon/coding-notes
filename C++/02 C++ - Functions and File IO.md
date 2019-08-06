# Functional C++ Crash Course

Author: methylDragon  
Contains a syntax reference for C++  
This time, we'll be going through Functions and File I/O!   
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=Rub-JsjMhWY

------

## Pre-Requisites

**Assumed knowledge (This is a C++ crash course, not a basic coding tutorial)**

- How **variables, loops, conditionals, etc**. work (Basic coding fundamentals will help a lot!)
- Linux (**Terminal/Console proficiency**) (We're going to need to compile our stuff)
- Gone through the all preceding parts of the tutorial



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Functional C++ Syntax Reference](#2)    
   2.1   [Functions](#2.1)    
   2.2   [Function Overloading](#2.2)    
   2.3   [Recursive Functions](#2.3)    
   2.4   [File I/O](#2.4)    
   2.5   [Lambda Functions](#2.5)    
   2.6   [Inline Functions](#2.6)    



## 1. Introduction <a name="1"></a>

We've gone through the basics of C++. Now let's throw in some functions and file interactions!



## 2. Functional C++ Syntax Reference <a name="2"></a>

### 2.1 Functions <a name="2.1"></a>

[go to top](#top)

Declare your functions **BEFORE** the main function!

`int addNumbers(int firstNum, int secondNum = 0)`
firstNum and secondNum are attributes. We set secondNum's default value if no value is given to be 0.

`int` on the `addNumbers` function is the RETURN value's datatype, in other words, the data type of the function's output.

```c++
int addNumbers(int firstNum, int secondNum = 0) {
	int combinedValue = firstNum + secondNum;
	
	return combinedValue;
	
}
```

> **Note**: combinedValue is not accessible anywhere else other than in addNumbers, as it is a **LOCAL** variable defined WITHIN a function.

**Calling Functions**

```c++
// So, we've made a function called addNumbers above
// Call it like this

addNumbers(1, 2); // It'll return 3

// Simple eh?
```

> **Note:**
>
> - When you write the data-type infront of the function name, you're defining a new function **prototype**
> - If you don't, you're calling it.
>
> Remember the distinction! 



#### **Returning arrays**

Read more + code source: https://www.geeksforgeeks.org/return-local-array-c-function/

Turns out you **can't return arrays** from functions just like that. It's not so simple.

This is a little premature since we have to go into OOP and/or pointers to do it.. but.. Here are the several ways to "return an array" from a function.

**Return a dynamically allocated array pointer**

Like so: `int *arr = new int[100];`

```c++
int *fun()
{
   int *arr = new int[100]; // HERE IT IS!

   arr[0] = 10;
   arr[1] = 20;
    
   return arr; // Return the pointer to the array!
}
 
int main()
{
    int *ptr = fun();
    cout << ptr[0] << " " << ptr[1]; // Prints 10 20
    return 0;
}
```

**Return a static array**

Like so: `static int arr[100];`

```c++
int *fun()
{
   static int arr[100] // HERE IT IS!

   arr[0] = 10;
   arr[1] = 20;
    
   return arr; // Return the pointer to the array!
}
 
int main()
{
    int *ptr = fun();
    cout << ptr[0] << " " << ptr[1]; // Prints 10 20
    return 0;
}
```

**Use Structs** (Probably my preferred method)

```c++
struct arrWrap
{
   int arr[100];
};
 
struct arrWrap fun() // Function returns the struct
{
   struct arrWrap x;
 
   x.arr[0] = 10;
   x.arr[1] = 20;
    
   return x;
}
 
int main()
{
   struct arrWrap x = fun();
   cout << x.arr[0] << " " << x.arr[1]; // And you can access the struct members
   return 0;
}
```



### 2.2 Function Overloading <a name="2.2"></a>

[go to top](#top)

> C++ allows specification of more than one function of the same name in the same scope. These are called overloaded functions and are described in detail in Overloading. Overloaded functions enable programmers to supply different semantics for a function, depending on the types and number of arguments.

> For example, a print function that takes a string (or char *) argument performs very different tasks than one that takes an argument of type double. Overloading permits uniform naming and prevents programmers from having to invent names such as print_sz or print_d. The following table shows what parts of a function declaration C++ uses to differentiate between groups of functions with the same name in the same scope. - Microsoft

```c++
void sameFunction(int a){
  // do stuff;
}

int sameFunction(float a){
  // do something else;
}

void sameFunction(int a, float b){
  // do something different;
}
```

Functions can have the same name, but do DIFFERENT THINGS depending on what gets passed to them!



### 2.3 Recursive Functions <a name="2.3"></a>

[go to top](#top)

These are functions that call THEMSELVES. Trippy.

```c++
int getFactorial(int number){
	int sum;
	if(number == 1) sum = 1; //Yeah you can do this
	else sum = getFactorial(number - 1) * number;
	return sum;
)
```

This function returns the factorial of itself, and it keeps calling itself which results in it calling itself until it stops, then it resolves one by one till all are resolved.

The last function calls to go on the stack are resolved first! USE THE STACK! Play some Magic!



### 2.4 File I/O <a name="2.4"></a>

[go to top](#top)

File input output.

Remember to `#include <string>` and `#include <fstream>`

```c++
string dragonQuote = "Rawr. But I can talk. Sup."; // Define the string to be written

ofstream writer("dragonQuote.txt");  //Open a .txt file called dragonQuote, this is an OUTPUT filestream

if(! writer) { //Check to see if the filestream is open

	cout << "Error opening file" << endl;
	return -1; // Return -1 if failed

	)	else {

	writer << dragonQuote << endl; // Write dragonQuote to writer, which causes dragonQuote.txt to contain only dragonQuote 
	writer.close(); // Close the file

	}
}

ofstream writer2("dragonQuote.txt", ios::app); //Create a writer object that appends to dragonQuote.txt

// Open a stream to append to what's there with ios::app
// ios::binary : Treat the file as binary
// ios::in : Open a file to read input
// ios::trunc : Default
// ios::out : Open a file to write output

if(! writer2) { //Check to see if the filestream is open

	cout << "Error opening file" << endl;
	return -1; // Return -1 if failed

	)	else {

	writer2 << "\n -methylDragon" << endl; // Append  "\n -methylDragon" 
	writer2.close(); // Close the file

	}
	
	char letter;
	
	ifstream reader("dragonQuote.txt"); // Open an input filestream that reads dragonQuote.txt
	
	if(! reader){
		
		cout << "Error opening file" << endl;
		return -1;
		
	} else {
		
		for(int i = 0; ! reader.eof(); i++){ // Read until end of file
		
		reader.get(letter); // Get the next letter 
		cout << letter; // And print it
	
	}
	
	cout << endl;
	reader.close();
}
```



### 2.5 Lambda Functions <a name="2.5"></a>

Lambda functions are great for creating anonymous functions that are not meant for reuse.

Lambdas help to replace **functors.** That is, classes where `operator()` is hidden. So you can call the classes just like normal functions. They're normal classes that override operator, so they can be called like so `someFunctor(SOME_VALUES_IF_THE_FUNCTOR_TAKES_THEM);`

So before we can meaningfully go through lambdas, we need to go through functors.

**Functor**

So for example, this is an example of a functor class.

```c++
// Source: https://dev.to/sandordargo/lambda-expressions-in-c-4pj4
class IsBetweenZeroAndTen {
  public:
  bool operator()(int value) {
    return 0 < value && value < 10;
  }
};

// Usage
IsBetweenZeroAndTen some_functor;
some_functor(10); // This should return false
```

Some methods or functions take in functors, which forces you to define these. But it can get a little messy, but luckily with lambdas there is a better way!

#### **Lambda Syntax**

```c++
// This is the barebones lambda expression
[]() {}

// Whereas this is the full one
[]() mutable -> T { }

// Example
auto lambda = []() { cout << "Rawr" << endl; };
lambda(); // This prints Rawr
```

Where:

- `[ ]` is the capture list
- `( )` is the argument list
- `{ }` is the function body

#### **Capture List**

The capture list allows for the capture of variables (by reference or value) outside the actual lambda, and which are not passed in to the function that is defined!

```c++
// Capture nothing
[]

// Capture by value
[some_referenced_value]

// Capture by reference
[&some_referenced_value]

// Capture all variables in scope by value
[=]

// Capture all variables in scope by reference, with exclusions
[=, &but_me_by_reference, &me_too]

// Capture all variables in scope by reference
[&]

// Capture all variables in scope by reference, with exclusions
[&, but_me_by_value, me_too]

// Capture the surrounding object
[this]
```

#### **Argument List and Function Body**

Argument lists work exactly like standard C++ function argument lists. They define what variables are passed into the actual function body of the lambda.

Same thing for the function body.

#### **Return Type**

Specify the return type of the lambda by using the `->` operator.

You normally can omit this if there is only one return statement, since it's implicitly inferred. But if not, leave it in.

```c++
// So we can see that this lambda returns an int
[]() -> int {}
```

#### **Mutable**

If a lambda is marked `mutable`, it is able to mutate the values that are passed in by value.

#### **Putting Some Of It Together**

```c++
auto upperBound = 42;
[upperBound](int value) -> bool {
  return 0 < value && value < upperBound;
}
```

#### **Example Lambda Usage**

A particularly good usage of lambdas is in iteration.

```c++
#include <iostream>
#include <algorithm>
#include <vector>

vector<int> numbers {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

for_each(numbers.begin(),
         numbers.end(), 
         [] (int y)
         {
             cout << y << endl;
         });
```

Here's another example. Here we define the lambda and immediately use it.

```c++
int x = 4;
auto y = [&r = x, x = x+1]()->int {
            r += 2;
            return x+2;
         }();  // Updates x to 6, and initializes y to 7.
```



### 2.6 Inline Functions <a name="2.6"></a>

Inline functions help to avoid the overhead of function calls. When you compile the code, the entire call to an inline function is instead replaced with the code inside the function call itself.

They're especially good if used on small functions (usually one-liner function bodies.)

In order to define an inline function, just write `inline` in front of your function definition.

```c++
// Source https://www.geeksforgeeks.org/inline-functions-cpp/

#include <iostream> 
using namespace std; 
inline int cube(int s) 
{ 
    return s*s*s; 
} 
int main() 
{ 
    cout << "The cube of 3 is: " << cube(3) << "\n"; 
    return 0; 
} //Output: The cube of 3 is: 27 

// The inline function makes the function call line equivalent to
cout << "The cube of 3 is: " << 3 * 3 * 3 << "\n";
```

> Note, all functions defined inside a class body are inline functions. If you need the function to be explicitly inline, define the function prototype in the class, then declare the actual function definition outside the class with an inline.

```c++
// Source: https://www.geeksforgeeks.org/inline-functions-cpp/

class S 
{ 
public: 
    int square(int s); // declare the function 
}; 
  
inline int S::square(int s) // use inline prefix 
{ 
  
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