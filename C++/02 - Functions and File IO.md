```diff 

						   .     .
                        .  |\-^-/|  .    
                       /| } O.=.O { |\  
                      /´ \ \_ ~ _/ / `\
                    /´ |  \-/ ~ \-/  | `\
                    |   |  /\\ //\  |   | 
                     \|\|\/-""-""-\/|/|/
                             ______/ /
                             '------'
                _   _        _  ___                         
      _ __  ___| |_| |_ _  _| ||   \ _ _ __ _ __ _ ___ _ _ 
     | '  \/ -_)  _| ' \ || | || |) | '_/ _` / _` / _ \ ' \ 
     |_|_|_\___|\__|_||_\_, |_||___/|_| \__,_\__, \___/_||_|
                        |__/                |___/          
     -------------------------------------------------------
                        methylDragon.com                     
```

# C++ CRASH COURSE
# (SECTION I: THE BASICS)
<center>A brief summary of: https://www.youtube.com/watch?v=Rub-JsjMhWY</center>

## Comments
```c++
// this is a comment

/* multi
line comment! */
```

## Importing Libraries
`#include <NAME>`

common ones are

`<iostream> ` for cout, etc.
`<vector> ` for vectors
`<string>`  for strings
`<fstream>`  for file input outputs

##Hello World! (RAWR.cpp)

**Every C++ program consists of a main function**

```c++
int main() {
	std::cout << "Rawr"<<endl; // endl is carriage return (enter key)
	
	return 0;

} 
```
If you don't want to type std:: , then type

`using namespace std;  //right after your import statements`

##Running RAWR.cpp
`$ g++ -std=c++11 RAWR.cpp`

"Using g++, on version 11 of c++, COMPILE RAWR.cpp"

`$ ./a.out`

"RUN RAWR.cpp"

##Variables and Data Types

**Variables are like containers for data**

Variables must start off with a letter, but can contain numbers, or underscores. (But they MUST start off with a letter.)

Let's make a constant variable of type double (which means it's a decimal number that can store more numbers than the float type)

`const double PI = 3.1415926535;` DON'T FORGET YOUR semicolons!

Also, constants are usually declared with all caps

!! Note, when you declare variables outside of a function, they're **GLOBAL** they can be read by all functions. But if they're declared WITHIN a fucntion, they're **LOCAL** and can only be read by the function in question.

**Other variable data types you have are**
`char` (Stores a single character, a single byte)
`int`  (Whole numbers)
`float` (Decimals, accurate up to 6 .dp)
`boolean` (Normally starts with isName) (Contains TRUE or FALSE, i.e. 1 or 0)
`short int` (Int of 16 bits)
`long int` (int of 32 bits)
`long long int` (Int of 64 bits)
`unsigned int` (Same size as signed version)
`long double` (Not less than double)

### You can assign or change the values stored in a variable with
Just use the equals sign!
Make sure you remember your semi-colons and you ensure it's the right data-type!

`int varName = 5;`
`varName = 8;`

So you initialised varName as an integer containing 5
Then you changed the data stored in it to 8 instead.

#### Outputting variables
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

## Arithmetic Operations
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
cout << "5++ =  " << five++ << endl;
cout << "++5 = " << ++five << endl;
cout << "5-- = " << five-- << endl;
cout << "--5 = " <, --five << endl;
```

Returns
```c++
5++ = 5
++5 = 7
5-- = 7
--5 = 5
```

### That's so weird!

The reason for this is because when you increment or decrement on the **RIGHT** side, it'll perform the action **AFTER** it gives the value.

If you increment or decrement it on the **LEFT** side, it'll perform the action **BEFORE** it gives the value. The reason you have 7 is because 5++ incremented 5 to a 6, but sent 5 before it incremented, so when you have ++5, it took a 6 to begin with and incremented it to 7, then printed it.

### Here's another shorthand!

`five += 5; // five = five + 5;`
`five -= 5; //five = five - 5;`

### Order of operations
REMEMBER IT!

## Conditionals

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

**Example**

```c++
int age = 70;
bool isNotIntoxicated = true;

if((age >= 1) && (age < 16) { // If you're 1 or older, but less than 16, you can't drive

	cout << "You can't drive" << endl;

	} else if(! isNotIntoxicated) { // If you're not NotIntoxicated (i.e. that you are) you can't drive either

	cout << "You can't drive" << endl;
	}
}	
```

## Ternary Operators
Use these if you want to look cool!

Example

```c++
// variable = (condition) ? true : false

int largestNum =  (5>2) ? 5 : 2;
```

"Is 5 greater than 2? If yes, largestNum = 5, if no, largestNum = 2."



## Switch cases
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

## Arrays
Arrays store multiple values of the same datatype. They're like 'boxes in memory'.

`int myFavNums[5]`
You need to define how many boxes there are in the array in the beginning, and this can't be changed. (We'll use vectors to change them later on)

example

```c++
int myFavNums[5];

int badNums[5] = {4, 13, 14, 24, 34};

cout << "Bad NUmber 1: " << badNums[0] << endl; //Prints 4, as indexing begins from 0, so 4 is index 0, 13 is index 1, 14 is index 2, 24 is index 3, and so on
```

## Multi-Dimensional Arrays
"Boxes of boxes"

```c++
char myName[5][6] = {{'m','e','t','h','y','l'},{'D','r','a','g','o','n'}}

cout << "2nd letter in 2nd array " << myName[1][1] << endl; // prints 'r'

// You can change elements in an array like you do with variables as well!

myName[0][2] = 'e';

cout << myName[0][2] << endl; // prints e
```

## Strings

Strings are arrays of characters! An array of strings then, is essentially a two-dimensional array.

Always surround them by double quotes. In C++ strings are objects! (Unlike C)

`string methylDragon = "methylDragon";`
is equivalent to
`char methylDragon[1][12] = {'m','e','t','h','y','l','D','r','a','g','o','n'};`
or
`string methylDragon = str("methylDragon");`

```
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

## User Input

`getline(cin, storageVariable);`

If you want to get an integer, remember to use `intStorageVariable = stoi(storageVariable)`

## For loops

"Initialise an incrementer i
FOR as long as (some condition relating to i), RUN the code block
Each time you finish running the code block, INCREMENT according to your INCREMENTING statement"

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

## While loop

Use these when you don't know ahead of time when this loop is going to end.

```
int randNum = (rand() % 100) // generates numbers between 0 and 99

while(randNum != 99){
	cout << randNum << ", ";
	
	randNum (rand() % 100);
}

cout << endl;
```

It'll print until the condition is no longer fulfilled! So in this case we'll print random numbers until randNum assumes the value 99.

## Do While loops

Use them when you want to execute whatever is in the loop at least **ONCE**.

```
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

## Vectors
Vectors are like arrays, but with one key difference.

**THEIR SIZE CAN CHANGE**

```c++
vector <int> lotteryNumVect(10);`
int lotteryNumArray[5] = {4, 13, 14, 24, 34};

lotteryNumVect.nsert(lotteryNumVect.begin(), lotteryNumArray, lotteryNumArray+3); //This just appends the first three numbers from lotteryNumArray
```
Then, typing
`cout << lotteryNumVect.at(2) << endl;`
Prints "14" as "14" is in index 2

`lotteryNumVect.insert(lotteryNumVect.begin()+5, 44);` 
.begin puts you back to index 0, and +5 brings you to index 5. Then we insert 44 into the container at index 5 within lotteryNumVect.

`lotteryNumVect.push_back(64);` 
Adds 64 at the end of the vector.

`lotteryNumVect.back();` 
Returns the final value of the vector.

`lotteryNumVect.front();`
Returns the first value of the vector

`lotteryNumVect.size();`
Returns the size of the vector

`lotteryNumVect.pop_back();`
Removes the final value of the vector.

```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

# C++ CRASH COURSE
# (SECTION II: FUNCTIONS AND MORE)

## Functions
Declare your functions **BEFORE** the main function!

`int addNumbers(int firstNum, int secondNum = 0)`
firstNum and secondNum are attributes. We set secondNum's default value if no value is given to be 0.

int on the addNumbers function prototype is the RETURN datatype, in other words, the data type of the function's output.

```
int addNumbers(int firstNum, int secondNum = 0) {
	int combinedValue = firstNum + secondNum;
	
	return combinedValue;
	
}
```

!! NOTE. combinedValue is not accessible anywhere else other than in addNumbers, as it is a **LOCAL** variable defined WITHIN a function.

### You CAN overload functions
>C++ allows specification of more than one function of the same name in the same scope. These are called overloaded functions and are described in detail in Overloading. Overloaded functions enable programmers to supply different semantics for a function, depending on the types and number of arguments.

>For example, a print function that takes a string (or char *) argument performs very different tasks than one that takes an argument of type double. Overloading permits uniform naming and prevents programmers from having to invent names such as print_sz or print_d. The following table shows what parts of a function declaration C++ uses to differentiate between groups of functions with the same name in the same scope. - Microsoft


```
int addNumbers(int firstNum, int secondNum, int thirdNum){
	return firstNum + secondNum + thirdNum;
```

Calling the two functions

`cout << addNumbers(1) << endl;`
Prints 0

`cout << addNumbers(1, 5, 6)`
Prints 12

## Recursive Functions
These are functions that call THEMSELVES. Trippy.

```
int getFactorial(int number){
	int sum;
	if(number == 1) sum = 1; //Yeah you can do this
	else sum = getFactorial(number - 1) * number;
	return sum;
)
```

This function returns the factorial of itself, and it keeps calling itself which results in it calling itself until it stops, then it resolves one by one till all are resolved.

## FILE I/O
File input output.

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

## Exception Handling

LITERALLY THE BEST THING EVER

THIS IS PART OF WHAT WON ME THE GOOGLE HACKATHON. DEBUGGING LOVE.

```
try{ // Try this. If it doesn't work, then it's fine.

	if(number != 0){
	
		cout << 2/number << endl;
	
	} else throw(number); // Throw the problematic number

}

catch(int number) { // Catch it!
	
	cout << number << " is not valid" << endl; // Prints 0 (or any non number) is not valid
	
}
```

## POINTERS
#### OH NO