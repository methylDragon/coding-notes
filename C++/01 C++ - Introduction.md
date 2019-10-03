# Basic C++ Crash Course

Author: methylDragon  
Contains a syntax reference for C++  
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=Rub-JsjMhWY

------

## Pre-Requisites

**Assumed knowledge (This is a C++ crash course, not a basic coding tutorial)**

- How **variables, loops, conditionals, etc**. work (Basic coding fundamentals will help a lot!)
- Linux (**Terminal/Console proficiency**) (We're going to need to compile our stuff)
- Gone through the all preceding parts of the tutorial



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Basic C++ Syntax Reference](#2)    
   2.1   [Comments](#2.1)    
   2.2   [Importing Libraries](#2.2)    
   2.3   [Hello World!](#2.3)    
   2.4   [Running Programs](#2.4)    
   2.5   [Variables and Data Types](#2.5)    
   2.6   [Ouputting Variables and Variable Info](#2.6)    
   2.7   [Casting (Type Conversion)](#2.7)    
   2.8   [Arithmetic Operations](#2.8)    
   2.9   [Conditionals](#2.9)    
   2.10 [Ternary Operators](#2.10)    
   2.11 [Switch Cases](#2.11)    
   2.12 [Arrays](#2.12)    
   2.13 [Multi-Dimensional Arrays](#2.13)    
   2.14 [Strings](#2.14)    
   2.15 [User Input](#2.15)    
   2.16 [For Loops](#2.16)    
   2.17 [While Loops](#2.17)    
   2.18 [Do While Loops](#2.18)    
   2.19 [Vectors](#2.19)    
   2.20 [Printf](#2.20)    
   2.21 [Argc and Argv](#2.21)    
   2.22 [Exception Handling](#2.22)    
   2.22 [Scopes](#2.23)    
3. [Reference Links](#3)  




## 1. Introduction <a name="1"></a>

C++ is great for most stuff! It allows you to do memory management with it, and it's also a good place to start learning about concepts like references, pointers, etc.

More practically, it's used almost everywhere because of its speed, and if you're not using an offline app written by it, you're probably using stuff like Java, etc.

It's powerful, but hard. If you're not very familiar with programming, maybe don't start with this, but if you like a challenge... Well, here we go. I'll hold your hand a little along the way, but we're all learners here.

> You need a compiler to build and run C++ programs!
>
> I can only really help for Windows at the moment.
>
> Windows: Cygwin or MinDW: https://www3.ntu.edu.sg/home/ehchua/programming/howto/Cygwin_HowTo.html
>
> Linux should be Googleable though!

> For choice of editor, I'd go with:
>
> - Atom
> - Visual Studio Code
> - Sublime Text
>
> I haven't found a need for a full IDE like Eclipse yet, so...



## 2. Basic C++ Syntax Reference <a name="2"></a>

### 2.1 Comments <a name="2.1"></a>

[go to top](#top)

```c++
// this is a comment

/* multi
line comment! */
```



### 2.2 Importing Libraries <a name="2.2"></a>

[go to top](#top)

`#include <NAME>` or `#include "NAME"` !!

common ones are

`<iostream> ` for cout, etc.
`<vector> ` for vectors
`<string>`  for strings
`<fstream>`  for file input outputs



> Note on `< >` vs `" "`:
>
> The type of symbol you use when importing libraries affects how the compiler prioritises which library to import in the event of a clashing name.
>
> 
>
> `< >`: Search for the library/file in the include path list first    
>
> `" "`: Search for the library/file in the current directory of your source code
>
> 
>
> Generally, the convention is to always use `< >` for standard libraries, and `" "` otherwise.



### 2.3 Hello World! (RAWR.cpp) <a name="2.3"></a>

[go to top](#top)

**Every C++ program consists of a <u>main function</u>**

```c++
#include <iostream>

int main() {
	std::cout << "Rawr" <<endl; // endl is carriage return (enter key)
	
	return 0;
} 
```

If you don't want to type std:: , then type

`using namespace std;  //right after your import statements`



### 2.4 Running Programs (RAWR.cpp) <a name="2.4"></a>

[go to top](#top)

Using a Linux **Terminal** in the directory RAWR.cpp is stored

`$ g++ -std=c++11 RAWR.cpp`

Means: "Using g++, on version 11 of c++, COMPILE RAWR.cpp"

`$ ./a.out`

Means: "RUN RAWR.cpp" (a.out stands for Assembler Output. That is, the last thing output by the assembler.)



### 2.5 Variables and Data Types <a name="2.5"></a>

[go to top](#top)

**Variables are like containers for data**

Variable names can contain letters, underscores or numbers, but they **MUST NOT** start off with a number.
(If you start off writing your variable's name with number, your compiler will acknowledge that you're trying to write a number, not the variable's number!)

Let's make a constant variable of type double (which means it's a decimal number that can store more numbers than the float type)

`const double PI = 3.1415926535;` DON'T FORGET YOUR semicolons!

Also, constants are usually declared with all caps

!! Note, when you declare variables outside of a function, they're **GLOBAL** they can be read by all functions. But if they're declared WITHIN a function, they're **LOCAL** and can only be read by the function in question.

**Other variable data types you have are**
`char` (Stores a single character, a single byte)
`int`  (Whole numbers)
`float` (Decimals, accurate up to 6 .dp)
`bool` (Normally starts with isName) (Contains TRUE or FALSE, i.e. 1 or 0)
`short int` (Int of 16 bits)
`long int` (int of 32 bits)
`long long int` (Int of 64 bits)
`unsigned int` (Same size as signed version)
`long double` (Not less than double)

#### **decltype**

Declared type. You may be able to get the type of a variable by calling decltype on it. No promises though!

```c++
int n = 10;
decltype(n);
```



**Example Variable Initialisation:**

```c++
bool isTrue = true;
int myInt = 6;
char myChar = 'a';

// NOTE: Single Quotes ' declare characters
// Double quotes " declare strings (a null terminated array)
// So, if you did char myChar = "a"; It will throw an error! As what you're really saying is
// char myChar = {'a','\0'}; Which is too big for the array

// You can also initialise multiple variables OF THE SAME TYPE on the same line
int x = 6, y = 2;
```

> You can also use the `auto` keyword if you're not sure which type to use, or if the types might vary. Do note that this sometimes involves some overhead.
>
> It does type deduction, which can sometimes avoid implicit type conversions.
>
> ```c++
> auto my_auto = 1;
> ```



#### **Multiple ways to initialise a variable**

But it actually turns out that there are three ways to initialise a variable!

You **generally** (not always!) want to prioritise the way you initialise variables in this order, because as we go down, memory use becomes less efficient (assignment creates takes up more space in memory since memory is allocated to the variable (which will be empty) in addition to the data.)

- Brace initialisation (Supported from C++11 onwards) (aka uniform initialisation)
- Direct initialisation (Almost as good as brace, but not supported for some types)
- Copy Initialisation (Assignment) (Bad for memory, and also not supported for some types)

**Brace Initialisation**

Read more: https://mbevin.wordpress.com/2012/11/16/uniform-initialization/

```c++
int variable{5}; // Now stores 5
int empty_variable{}; // Empty

int error_variable{4.5}; // Brace initialisation disallows 'narrowing' type conversions
// You'll get an error!


int array_var[] {1, 2, 3, 4, 5}; // Oh look an array!
std::vector<int> vector_var {1, 2, 3, 4, 5}; // Oh my a vector! (Read about vectors later)
std::set<int> set_var {1, 2, 3, 4, 5}; // Check Python tutorial for set
std::map<int, std::string> map_var { {0, "zero"}, {1, "one"}, {2, "two"} }; // A 'DICT'!
```

**Direct Initialisation**

```c++
int variable(525); // Now stores 525

// Note: variable() will not work! You can't initialise an empty variable that way
```

**Copy Initialisation** (Lowest priority, try to avoid. Because this first generates the value, then copies it into the variable. It's a two step process that uses usually twice the memory and takes longer.)

```c++
// We've seen this before

int a = 5; // Woo
```



**You can assign or change the values stored in a variable**: Just use the equals sign!

Make sure you remember your semi-colons and you ensure it's the right data-type!

```c++
int varName = 5;
varName = 8;
```

So you initialised varName as an integer containing 5
Then you changed the data stored in it to 8 instead.

#### **Defining Custom Types**

You can define a type alias by using the `typedef` keyword!

```c++
// Simple typedef
typedef unsigned long ulong;
ulong var_five{5}; // So now we can declare an unsigned long using this type alias!

// Pointer typedef
typedef int* int_ptr;

int a = 5;
int_ptr b = &a;

// Struct typedef
typedef struct
{
    int a;
    int b;
    int c;
} my_struct;

my_struct haha;
haha.a = 1;
haha.b = 2;
haha.c = 3;
```



### 2.6 Outputting Variables and Variable Info <a name="2.6"></a>

[go to top](#top)

```c++
float favNum = 3.141592;

cout << "Favourite number" << favNum << endl;
```

Prints `Favourite Number3.14159`

**To find out the number of bytes for a datatype,**

```c++
cout << "Size of int " << sizeof(someInt) << endl;
```

Prints `Size of int 4`

!! If you go out of of the byte range, you'll get overflow (google that, basically bad stuff, your number will be inaccurate)



### 2.7 Casting (Type Conversion) <a name="2.7"></a>

[go to top](#top)

```c++
// You can convert between data types using casting!
cout << "4 / 5 = " << 4 / 5 << endl;
cout << "4 / 5 = " << (float) 4 / 5 << endl; // See how it works?
```



### 2.8 Arithmetic Operations <a name="2.8"></a>

[go to top](#top)

```c++
+ //add
-  //subtract
* //multiply
/ //divide
% //modulus (returns remainder of division)
++ //increments 1 (+1)
-- //decrements 1 (-1)
```

Example

```c++
int five = 5;
cout << "5++ gives  " << five++ << endl; // Increment by 1
cout << "++5 gives " << ++five << endl; // Increment by 1
cout << "5-- gives " << five-- << endl; // Decrement by 1
cout << "--5 gives " << --five << endl; // Decrement by 1
```

Returns

```
5++ gives 5
++5 gives 7
5-- gives 7
--5 gives 5
```

> **Wait a sec, weird behaviour!**
>
> The reason for this is because when you increment or decrement on the **RIGHT** side, it'll perform the action **AFTER** it gives the value.
>
> If you increment or decrement it on the **LEFT** side, it'll perform the action **BEFORE** it gives the value. The reason you have 7 is because 5++ incremented 5 to a 6, but sent 5 before it incremented, so when you have ++5, it took a 6 to begin with and incremented it to 7, then printed it.
>
> **Order of operations! Remember it!**

**Another shorthand!**

`five += 5; // five = five + 5;`
`five -= 5; // five = five - 5;`



### 2.9 Conditionals <a name="2.9"></a>

[go to top](#top)

**Comparison Operators**

```c++
== // equal to 
!= // NOT equal to
> // more than
< // less than
>= //  more than or equal to
<= // less than or equal to
```

!! NOTE. "==" IS **NOT** "="

== COMPARES
= ASSIGNS VALUES

**Logical Operators**

```c++
&& // AND
|| // OR
! // NOT
```

**Example IF, ELIF**

```c++
int age = 70;
bool isNotIntoxicated = true;

if((age >= 1) && (age < 16) { // If you're 1 or older, but less than 16, you can't drive

	cout << "You can't drive" << endl;

	} else if(! isNotIntoxicated) { // If you're not NotIntoxicated (i.e. that you are) you can't drive either

	cout << "You can't drive" << endl;
	} // else is not stated here, since I'm just defining nothing for it
}	
```



### 2.10 Ternary Operators <a name="2.10"></a>

[go to top](#top)

Use these if you want to look cool!

Example

```c++
// variable = (condition) ? true : false

int largestNum =  (5>2) ? 5 : 2;
```

"Is 5 greater than 2? If yes, largestNum = 5, if no, largestNum = 2."



### 2.11 Switch cases <a name="2.11"></a>

[go to top](#top)

Use it whenever you have a limited number of options

```c++
int greetingOption = 2;

switch(greetingOption) {

	case 1:
		cout << "Bonjour" << endl;
		break;
	case 2:
		cout << "Rawr" << endl;
		break;
	case 3:
		cout << "Raa" << endl;
		break;
	default:
		cout << "Sup" << endl;
```

In this case, `Rawr` will be printed



### 2.12 Arrays <a name="2.12"></a>

[go to top](#top)

Arrays store multiple values of the same datatype. They're like 'boxes in memory'.

`int myFavNums[5]`
You need to define how many boxes there are in the array in the beginning, and this can't be changed. (We'll use vectors to change them later on)

Example:

```c++
int myFavNums[5];

int badNums[5] = {4, 13, 14, 24, 34};

cout << "Bad Number 1: " << badNums[0] << endl; //Prints 4, as indexing begins from 0, so 4 is index 0, 13 is index 1, 14 is index 2, 24 is index 3, and so on
```



### 2.13 Multi-Dimensional Arrays <a name="2.13"></a>

[go to top](#top)

"Boxes of boxes"

```c++
char myName[2][7] = {{'m','e','t','h','y','l','\0'},{'D','r','a','g','o','n','\0'}}

cout << "2nd letter in 2nd array " << myName[1][1] << endl; // prints 'r'

// You can change elements in an array like you do with variables as well!

myName[0][1] = 'e';

cout << myName[0][1] << endl; // prints e
```



### 2.14 Strings <a name="2.14"></a>

[go to top](#top)

Strings are arrays of characters! An array of strings then, is essentially a two-dimensional array.

Always surround them by double quotes. In C++ strings are objects! (Unlike C. So think Object-Oriented Programming!)

`string methylDragon = "methylDragon";`
is equivalent to
`char methylDragon[1][12] = {'m','e','t','h','y','l','D','r','a','g','o','n'};`
or
`string methylDragon = str("methylDragon");`

```c++
//Assigning to C++ string
methylDragon = "Rawr";
str1 = str2.assign(str2); // Now str1 = str2
str1 = str2.assign(str2, 0, 5); // Pull 5 characters, starting from index 0. Now str1 contains the FIRST FIVE CHARACTERS of str2

//Concatenating two C++ strings
str1 += str2;
str = str1 + str2;

//Copying a C++ string
str = otherString;

//Finding the length of a C++ string
str.length();
str.size();

//Appending things to strings
str.append("stuff to append");

//Checking if a string is empty
str.empty() // Returns 0 is no, 1 if yes

//Accessing a single character
str[index];
str.at(index);
str(index, count);

//Output a string
cout << str;
cout << setw(width) << str;

//Comparing strings
str1.compare(str2); // If str1 == str2, returns 0,  if str1 > str2, returns 1, if str1 < str2. This is all according to alphabeticals

//Searching a string
int stringIndex = string.find("FINDTHIS", 0); // 0 is the index you want to start searching from.

//Inserting into a string
str1.insert(5, str2); //Inserts str2 into str1 from index 5

//Erasing parts of a string
str1.erase(6,7); // Starting from index 6, delete 7 characters.

//Replacing parts of a string
str1.replace(6, 5, "Maximus") //Starting from index 6, delete 5 characters, then append the string "Maximus"
```



### 2.15 User Input <a name="2.15"></a>

[go to top](#top)

`getline(cin, storageVariable);`

If you want to get an integer, remember to use `int intStorageVariable = stoi(storageVariable)`

(stoi stands for String to Integer. It's a function that converts the datatype of a `string` containing numbers to `integer`.)

```c++
// Example use
// Getline inputs the first sentence (until the first newline)

int quiz_marks;

printf("Enter your quiz marks!\n");
getline(cin, quiz_marks); // C++ will wait for user input via console

// Alternative form
// Though, note! This inputs the first word/number (till the whitespace)

printf("Enter your quiz marks!\n");
cin >> quiz_marks
```

```c++
// You can also do INPUT VALIDATION!

// Validation function
bool isNumber(const string &line) 
{
    if (line == "0") return 1; // If input is 0, return True
    if (atoi(line.c_str()) > 100){
        printf("You can't score more than 100!\n");
        return 0; // If input is more than 100, return False
    }
    return (atoi(line.c_str())); // If input has numbers above 0, return True
  // atoi means (alphanumeric to integer)
  // c_str: http://www.cplusplus.com/reference/string/string/c_str/
}

// Keep asking until a valid response is attained
do { // Do While loops explained later
	printf("Enter your final marks!\n");
	getline(cin, final_marks); // Wait for user input
	if (cin.good() && !isNumber(final_marks)) printf("ERROR!!! Enter a valid number please!\n");
} while (cin.good() && !isNumber(final_marks)); // While input is ok, and isNumber returns false, keep asking for final marks
```

> Notes on the differences between cin and getline(cin, <storagevariable>)
>
> Basically, `std::cin` (or more generally, any `std::istream`) is used directly in order to obtain formatted input, e.g. `int x; std::cin >> x;`.  `std::cin.getline()` is used simply to fill a raw `char *` buffer.
>
> Source: https://stackoverflow.com/questions/4745858/stdcin-getline-vs-stdcin



### 2.16 For loops <a name="2.16"></a>

[go to top](#top)

> Initialise an incrementer i
>
> **FOR** as long as (some condition relating to i), RUN the code block
>
> Each time you finish running the code block, INCREMENT according to your INCREMENTING statement

```c++
for(int i = 1; i <= 10; i++) { // Begin with i = 1, FOR as long as i is less than or equal to 10, run the code block, printing the value of i for that iteration, then let i = i + 1
	cout << i << endl; // prints 1 to 10
}

for(int j = 0; j < 6; j++) {
	for(int k = 0; k < 6; k++){
		cout << myName[j][k];
	}
}

// The above bit prints out meeylDragon (cause we changed t to e previously)
```

#### **A more Pythonic for loop**

```c++
int my_nums[5] = {1, 2, 3, 4, 5};

// Python equivalent:
// for x in my_nums:

for (int x: my_nums){
    cout << x << endl;
}

/* Output:
1
2
3
4
5
*/
```



### 2.17 While loops <a name="2.17"></a>

[go to top](#top)

Use these when you don't know ahead of time when this loop is going to end.

```c++
int randNum = (rand() % 100) // generates numbers between 0 and 99

while(randNum != 99){
	cout << randNum << ", ";
	
	randNum (rand() % 100);
}

cout << endl;
```

It'll print until the condition is no longer fulfilled! So in this case we'll print random numbers until randNum assumes the value 99.



### 2.18 Do While loops <a name="2.18"></a>

[go to top](#top)

Use them when you want to execute whatever is in the loop at least **ONCE**.

```c++
do {
	string numberGuessed;
	int intNumberGuessed;
	
	cout << "Guess between 1 and 10: ";
	
	getline(cin, numberGuessed); // Allow user input via keyboard, via cin, store the input in numberGuessed

	intNumberGuessed = stoi(numberGuessed) // stoi: string to integer, stod: string to double
	
	cout << intNumberGuessed << endl;	
} while(intNumberGuessed != 4);

cout << "You win!" << endl;
```

That's a guessing game that'll keep looping until you guess 4.



### 2.19 Vectors <a name="2.19"></a>

[go to top](#top)

Vectors are like arrays, but with one key difference.

**THEIR SIZE CAN CHANGE**

```c++
vector <int> lotteryNumVect(10);
int lotteryNumArray[5] = {4, 13, 14, 24, 34};

lotteryNumVect.insert(lotteryNumVect.begin(), lotteryNumArray, lotteryNumArray+3); //This just appends the first three numbers from lotteryNumArray
```

Then, typing

```c++
cout << lotteryNumVect.at(2) << endl;
// Prints "14" as "14" is in index 2

lotteryNumVect.insert(lotteryNumVect.begin() + 5, 44); 
// Inserts 44 at the 5th index 
// .begin puts you back to index 0, and +5 brings you to index 5. Then we insert 44 into the container at index 5 within lotteryNumVect.

lotterNumVect.insert(lotteryNumVect.begin(), lotteryNumArray, lotteryNumArray + 3)
// Inserts from the beginning of lotteryNumVect, the elements between lotteryNumArray[0] and lotteryNumArray[3], BUT NOT INCLUDING lotteryNumArray[3]

lotteryNumVect.push_back(64); 
// Adds 64 at the end of the vector.

lotteryNumVect.back(); 
// Returns the final value of the vector.

lotteryNumVect.front();
// Returns the first value of the vector

lotteryNumVect.size();
// Returns the size of the vector

lotteryNumVect.pop_back();
// Removes the final value of the vector.
```



### 2.20 Printf() <a name="2.20"></a>

[go to top](#top)

We've talked about `cout <<` when we were talking about printing things. But this is another way to do it that's kind of flexible as well!

Example from http://www.cplusplus.com/reference/cstdio/printf/

```c++
/* printf example */
#include <stdio.h>

int main()
{
   printf ("Characters: %c %c \n", 'a', 65);
   printf ("Decimals: %d %ld\n", 1977, 650000L);
   printf ("Preceding with blanks: %10d \n", 1977);
   printf ("Preceding with zeros: %010d \n", 1977);
   printf ("Some different radices: %d %x %o %#x %#o \n", 100, 100, 100, 100, 100);
   printf ("floats: %4.2f %+.0e %E \n", 3.1416, 3.1416, 3.1416);
   printf ("Width trick: %*d \n", 5, 10);
   printf ("%s \n", "A string");
   return 0;
}

/* OUTPUT:
Characters: a A
Decimals: 1977 650000
Preceding with blanks:       1977
Preceding with zeros: 0000001977
Some different radices: 100 64 144 0x64 0144
floats: 3.14 +3e+000 3.141600E+000
Width trick:    10
A string
*/
```

What's essentially happening is that the string elements preceded by a % act as standins where you can fill data! The data filled in here were constants, but you can potentially put variables in here too!

There's a whole bunch of different types, but the commonly used ones are

%c: character

%d: signed integer

%f: float

%s: string



### 2.21 Argc and Argv <a name="2.21"></a>

[go to top](#top)

Now that we've gone through a lot of C++, you should be curious why the main function is int main(). Why are there parentheses for it? Can it take arguments?

Turns out, it can. The function prototype for the main function is actually `int main(int argc, string argv[])`. This means that you can actually input arguments into the program when you're running it! Either from the command line or programatically.

argc stands for **argument count** (The default value is 1, the name of the program being run)

argv stands for **argument vector** (In this case, an array of strings. Each string is one of the arguments passed to the program.)

You can reference and access these variables! Notice how in the example how the first argument is the name of the program though!

Illustrative example:

```c++
int main(int argc, string argv[])
{
  int i;
  printf("Main has %d arguments inputted \n", argc);
  
  for(i=0; i < argc; ++i)
  {
    printf("%s\n", argv[i]);
  }
  return 0;
}

// Let's say... we call this program test.cpp
// And we run it using
// ./test ra raa rawr

/* OUTPUT:
Main has 4 arguments inputted
./test
ra
raa
rawr
*/
```



### 2.22 Exception Handling <a name="2.22"></a>

[go to top](#top)

LITERALLY THE BEST THING EVER. DEBUGGING LOVE.

```c++
try{ // Try this. If it doesn't work, then it's fine.

	if(number != 0){
	
		cout << 2/number << endl;
	
	} else throw(number); // Throw the problematic number

}

catch(int number) { // Catch it!
	
	cout << number << " is not valid" << endl; // Prints 0 (or any non number) is not valid
	
}
```



### 2.23 Scopes <a name="2.23"></a>

[go to top](#top)

Eh, you'll come across this sooner or later. So might as well let it be now.

Source: http://en.cppreference.com/w/cpp/language/scope

I'll just run through the block scope first since it's the easiest to illustrate. But basically, { } limits the name scope

```c++
int main()
{
    int a = 0; // scope of the first 'a' begins
    ++a; // the name 'a' is in scope and refers to the first 'a'
    
    {
        int a = 1; // scope of the second 'a' begins
                   // scope of the first 'a' is interrupted
        a = 42;    // 'a' is in scope and refers to the second 'a'                 
    } // block ends, scope of the second 'a' ends
      
    //             scope of the first 'a' resumes
} // block ends, scope of the first 'a' ends

int b = a; // Error: name 'a' is not in scope
```



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\
```

​    

---

 [![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)
