# PHP Syntax Reference

Author: methylDragon  
Contains a syntax reference for PHP  
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=7TF00hJI78Y

---

## Pre-Requisites

### Assumed knowledge (This is a syntax reference, not a basic coding tutorial)

- How **variables, loops, conditionals, etc**. work
- Basic **HTML**

### Good to know

- **Regular Expressions** (Regex) (It's more or less a souped up version of Ctrl+F)
  - https://regexone.com/ Get your refresher here



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Basic Syntax Reference](#2)    
   2.1 [HTML and PHP](#2.1)    
   2.2 [Comments](#2.2)    
   2.3   [Variable Declaration](#2.3)    
   2.4   [Echo](#2.4)    
   2.5   [Heredoc (Multiline strings)](#2.5)    
   2.6   [Arithmetic](#2.6)    
   2.7   [References](#2.7)    
   2.8   [Conditionals and Switch Cases](#2.8)    
   2.9   [Loops](#2.9)    
   2.10 [Functions](#2.10)    
   2.11 [Arrays](#2.11)    
   2.12 [Array Key Assignment](#2.12)    
   2.13 [Array Functions](#2.13)    
   2.14 [String Functions](#2.14)    
   2.15 [Printf](#2.15)    
   2.16 [Regular Expressions](#2.16)    
   2.17 [Die](#2.17)  
3. [Using PHP with MySQL](#3)    
   3.1 [Establishing Database Connection](#3.1)    
   3.2 [Database Functions](#3.2)    
   3.3 [Querying the Database](#3.3)    
   3.4 [Altering the Database](#3.4)    
   3.5 [Small note on ->](#3.5)  
4. [Reference Links](#4)  




## 1. Introduction <a name="1"></a>

> PHP can be used for lots of things too, but the main thing its used for is generating html on the fly. 
>
> So you know how a webpage works in the really simple sense, right? A html file has some text in it that includes words to put on the page, instructions on how to display those words, links to other websites, locations of images to use in the page, etc.
>
> So imagine reddit was just done with regular html. This page would be an html file, but when I post this comment, it'd have to delete the html file and create a new one with my comment added. And it'd have to do the same thing when I upvote it. The biggest problem would be, what happens if someone clicks on a link to the page after you delete it but before you write the new one? And then how to you keep track of the order of things for stuff like what gets in the front page? It'd be way too complicated.
>
> Now imagine reddit is done with PHP and mySQL (no idea if it is, but it's something similar). To display this page, you have a php file that's like a fill in the blanks. Think of the php for the top of this reddit page like this:
>
> ```
> <title>
> submitted <time> ago by <username>
> <text>
> <number> comments
>
> ```
>
> And so on. PHP gets executed by the server before it sends the html to your browser, and it will have hooks in it to a MySQL database, to go and look up the info to fill in the blanks. It would look up other info like the number of up and down votes, the number of comments, figure out how to display the comments all nifty and threaded like it does, and stuff like that. (r/eli5)

Additionally, you can use PHP to push and pull from a MySQL database, as well as pair it with forms! WordPress also uses PHP in its 'content generation loop'.

NOTE: The parenthesis formatting I'm using here is for PHP. It differs from WordPress in that here I won't care about the whitespace after the opening parenthesis and before the closing parenthesis. DO NOT FORGET IT FOR WordPress!



## 2. Basic Syntax Reference <a name="2"></a>

### 2.1 HTML and PHP <a name="2.1"></a> 

[go to top](#top)

You can insert PHP into a HTML file by using the `<?php` and `?>` tags! HTML and PHP can then be put in the same file in this same way.

```php+HTML
<!-- This outputs "Hello, World!" to the screen. -->
<html>
	<head>
		<title>Hello World</title>
	</head>
	<body>
		<?php
      		echo "Hello, World!";
      	?>
	</body>
</html>
```



### 2.2 Comments <a name="2.2"></a> 

[go to top](#top)

```php
// This is a single line PHP comment
# This is a single line PHP comment

/*
This is a multiline PHP comment
*/

<!-- This is a multiline HTML comment -->

```



### 2.3 Variable Declaration <a name="2.3"></a> 

[go to top](#top)

```php
// Use a $ sign to declare a variable

	$myArray = array("dragons", "say", "rawr");
	$myInt = 12;
	$myNull;

# Note: Variable names are CASE-SENSITIVE, but TYPES are implicit
# Each variable's first character has to be alphabetical

// Cosntant declaration
	define('PI', 3.1415926)

```



### 2.4 Echo <a name="2.4"></a> 

[go to top](#top)

```php
// Echo prints the thing in quotes to screen
// This prints: Data processed (as a string)
	echo "<p>Data processed</>";

// If you want to print double quotes, escape them with \ (Note, single quotes ignore escape sequences)
// This prints: "Data processed" (as a string)
	echo "<p>\"Data processed\"</>";

# Note, you can't print out variables directly like that in the string, since the double quotes specify that you're printing that as a string

// If you want to print variables out directly alongside strings, use string concatenation
// This prints <someVar> Hi!
	echo $someVar . " Hi!";
```

Intermediary Note: **DO NOT FORGET YOUR SEMICOLON**



### 2.5 Heredoc (Multiline strings) <a name="2.5"></a> 

[go to top](#top)

"Treat everything as a literal until the end of data identifier is parsed"

```php
// Sounds scary, but actually...
// It's a way to do multiline strings! Encapsulate the string with <<<[SOMEIDENTIFIER] and [SOMEIDENTIFIER]; Standard way is <<<EOD and EOD; (EOD stands for End of Data)
	$multilineString = <<<EOD
I can type anything I want, even "This" and "That".
I can even say RAWR without any heed!
The lines can just go on and on until...
EOD;

# NOTE: It is important that the ending identifier contains NO OTHER CHARACTERS, THIS INCLUDES INDENTATIONS before the identifier, and WHITESPACES after the semicolon
```



### 2.6 Arithmetic and Type Arithmetic <a name="2.6"></a> 

[go to top](#top)

```php
// This is fairly self-explanatory if you can do basic math
	echo "</br> 5 + 2 = " . (5 + 2);
	echo "</br> 5 - 2 = " . (5 - 2);
	echo "</br> 5 * 2 = " . (5 * 2);
	echo "</br> 5 / 2 = " . (integer)(5 / 2); # Turns it into an int!
	echo "</br> 5 % 2 = " . (5 % 2) . "</br>";

// Shortcuts
	$someNum = 5;
	$someNum += 10; # Adds 10 to someNum
	# You can also use these!
	# += , -= , *= , /= , %=

// Single Increments and Decrements
	$number = 5;
	$number++; # Echoing this now will give 5, since it increments AFTER any access
	++$number; # Echoing this will give 7, since it increments BEFORE any access
```



### 2.7 References <a name="2.7"></a> 

[go to top](#top)

```php
// Think C++ Think Aliases
// You can 'clone' variables this way
	$someNum = 100;
	$refToNum = &$randNum;
	echo '$refToNum = ' . $refToNum; # Prints $refToNum = 100
```



### 2.8 Conditionals and Switch Cases <a name="2.8"></a> 

[go to top](#top)

Comparison Operators

`==`, `!=`, `<`, `>`, `<=`, `>=` The standard stuff

`===` "True if the two values are **EQUAL** and of the **SAME TYPE**"

`!==` "True if two values are **NOT EQUAL** or the **SAME TYPE**

```php
// If, Else, Elif
// Use: if, else, elif
// && (AND), || (OR) also work
	if (($number == 10) && ($methylDragon == "Great")) {
		echo 'Woo!';
    } elseif (($number == 5) && ($methylDragon == "Awesome")) {
      echo 'Rawr!';
    } else {
      echo "Subscribe to methylDragon's Spotify anyway";
    }

// Ternary Operators also supported!
// condition ? <value if true>:<value if false>
	$biggestNum = (15 > 10) ? 15 : 10;

// Switch Cases
// For when you have a limited number of possible values
	switch($methylDragon) {
		case "methylDragon" :
			echo "Hello methylDragon!";
			break;
		case "ethylDragon" :
			echo "NO KNOCKOFF METHYLS!";
			break;
		default :
			echo "Hello!";
			break;
	}
```


### 2.9 Loops <a name="2.9"></a> 

[go to top](#top)

```php
// While loop
	$num = 0;
	while($num < 20) {
		echo ++$num . ',';
	}

// For loop (Initialise; Condition; Increment)
	for($num = 1; $num <= 20; $num++) {
		echo $num;
      
      	if ($num != 20) {
			echo ', ';
        } else {
			break;
        }
}
```



### 2.10 Functions <a name="2.10"></a> 

[go to top](#top)

```php
// I'm going to write down a function prototype now
	function addNumbers($num1, $num2) {
		return $num1 + $num2;
	}

// To call it use addNumbers()
	addNumbers(1,2); # Returns 3
```



### 2.11 Arrays <a name="2.11"></a> 

[go to top](#top)

```php
// Let's revisit the array we declared in 2.1
	$myArray = array("dragons", "say", "rawr");

	echo $myArray[0]; # Prints dragons

// Iterate through each array element
// Like python: for element in enumerate(<somearray)
// Prints dragons say rawr
	foreach($myArray as $word) {
		echo $word . ' ';
	}

// Multidimensional arrays are supported!
// Indexing is like C++ and Python: arrayName[$row][$col]
	$customers = array(array('Derek', '123 Main', '15212'),
                      array('Sally', '124 Main', '15213'))
```



### 2.12 Array Key Assignment <a name="2.12"></a> 

[go to top](#top)

```php
/* More or less like dictionaries!

You can assign a key to an array using =>
*/
	$dragons = array('Name'=>$dragonName, 'Subspecies'=>$dragonType);
```



### 2.13 Array Functions <a name="2.13"></a> 

[go to top](#top)

```php
// Here are some common handy functions to use!

// sort(): Sorts in ascending alphabetical order
	sort($myArray);
	sort($myArray, SORT_NUMERIC); # For numbers
	sort($myArray, SORT_STRING); # Treats the string like an array of characters

// asort(): Sorts arrays, keeping key-data pairs together
	asort($myArray);

// ksort(): Sorts arrays BY the keys
	ksort($myArray)
      
# Put an r infront of any of the functions to sort in reverse order
# Eg: rsort(), rasort(), rksort()
      
// Array alterers! array_function($array, [value1, value2, ..])
// array_pop(): Pops the LAST element off an array, returning it and shortening the array by an element
// array_shift(): Pops the FIRST element off an array, returning it and shortening the array by an element
// array_push(): Adds an element after the LAST element of an array
// array_unshift(): Adds an element before the FIRST element of an array

```



### 2.14 String Functions <a name="2.14"></a> 

[go to top](#top)

``` php
// I already noted down how to declare a string
	$rawrString = '     Raawr     ';

// Use strlen() to count the number of characters in a string
// Use ltrim to remove all left whitespace
// Use rtrim to remove all right whitespace
// USe trim to remove all whitespace on left and right
	echo strlen($rawrString); # Prints 15
	echo strlen(ltrim($rawrString)); # Prints 10
	echo strlen(rtrim($rawrString)); # Prints 10
	echo strlen(trim($rawrString)); # Prints 5

// Use strtoupper() to convert every letter to UPPERCASE
// Use strtolower() to convert every letter to lowercase
// User ucfirst() to capitalise each first letter

// Use explode() to convert a string to an array
	$rawrString = "Raa wr";
	$rawrArray = explode(' ', $rawrString, 2); 
	# ' ' is the DELIMITER
	# (optional) 2 is the number of pieces to include
	# Leftovers are unexploded, but included as the last element
	array_pop($rawrArray);
	$rawrRemadeString = implode(' ', $rawrArray, 2);
	# ' ' here is the JOINER
	# (optional) 2 is the number of pieces to include
	# Leftovers are discarded
	echo $rawrRemadeString; # Prints Raa (as wr was popped)

// substr() allows for string truncation
	$rawrSubString = substr("Raawr", 0, 2); # Now becomes Raa

// strcmp() compares strings
	strcmp("Man", "Manhole"); 
	# Returns -4, as Man is less than Manhole. It'll return 	# positive if Man is higher (alphabetical order)
	strcasecmp("Man", "Manhole"); # To ignore case

// strstr() allows for string truncation via search
// stristr() is the case-insensitive version of strstr()
	strstr("Raa Rwar Rawr", "Rwar"); # Returns Rwar Rawr
	stristr("Raa Rwar Rawr", "rwar"); # Returns Rwar Rawr

// strpos() returns the location of a match
	strpos("Raa Rwar Rawr", "Rwar"); # Returns 3 (ala array index)

// str_replace() allows you to replace stuff!
	str_replace("Rwar", "Rar", "Raa Rwar Rawr"); # Returns Raa Rar Rawr
```



### 2.15 Printf <a name="2.15"></a> 

[go to top](#top)

```php
// Use printf() like you would in C.
// Print variables without concatenation!
	$rawrString = "     Raawr     ";
	printf("The rawrString is %s", trim($rawrString));
	# Prints: The rawrString is Raawr

// You can use it for more, of course, like formatting stuff!
	$decimalNum = 2.3456;
	printf("Decimal num = %.2f", $decimalNum);
	# Prints: Decimal num = 2.35
```



### 2.16 Regular Expressions <a name="2.16"></a> 

[go to top](#top)

```php
// Use preg_match() to find and store matches
	preg_match('/abc/', $string);
	# Returns true if 'abc' is in $string
	preg_match('/abc/', $string, $matches);
	# Fills $matches with matches. Useful for more complicated searches
	
// Use preg_replace() to replace matches
	$result = preg_replace('/abc/', 'def', $string);   
	# Replace all 'abc' with 'def'
```



### 2.17 Die <a name="2.17"></a> 

[go to top](#top)

```php
// Sounds morbid, but it's an alias of exit()
// You use it to exit the script to prevent unresponsive scripts
	die("Error message here");
```



## 3. Using PHP with MySQL <a name="3"></a>

### 3.1 Establishing Database Connection <a name="3.1"></a> 

[go to top](#top)

*** mysqli is used instead of mysql because it's the improved interface**

It's good to save this as a file and then call it once elsewhere

**mysqli_connect.php**

```php
<?php
// Opens a connection to the database
// Since it is a php file it won't open in a browser 
// It should be saved outside of the main web documents folder
// and imported when needed

/*
Command that gives the database user the least amount of power
as is needed.
GRANT INSERT, SELECT, DELETE, UPDATE ON test3.* 
TO 'user'@'localhost' 
IDENTIFIED BY 'password';
SELECT : Select rows in tables
INSERT : Insert new rows into tables
UPDATE : Change data in rows
DELETE : Delete existing rows (Remove privilege if not required)
*/

// Defined as constants so that they can't be changed
DEFINE ('DB_USER', 'user');
DEFINE ('DB_PASSWORD', 'password');
DEFINE ('DB_HOST', 'localhost');
DEFINE ('DB_NAME', 'test');

// $dbc will contain a resource link to the database
// @ keeps the error from showing in the browser

$dbc = @mysqli_connect(DB_HOST, DB_USER, DB_PASSWORD, DB_NAME)
OR die('Could not connect to MySQL: ' . mysqli_connect_error());
?>
```

**Connect using this code snippet**

```php
// Get a connection for the database (Require once because you only need to connect once)
	require_once('../mysqli_connect.php');
```



### 3.2 Database Functions <a name="3.2"></a> 

[go to top](#top)

Remember each column denotes a property! Each row denotes an entry!

```php
# Note: $cxn was specified as $dbc in 3.1

// Establish database connection
	mysqli_connect("host","accnt","passwd");

// Close database connection. Don't forget to close the connection!!
	mysqli_close($cxn);
      
// Select database
	mysqli_select_db($cxn, "dbname",);

// Query database (More on queries later)
	mysqli_query($cxn,"query");

// Fetch result row as associative array. Column names are keys storing values
// It'll iterate through until there are no more result rows left
// $result comes from a call from mysqli_query() or mysql_query 
	mysqli_fetch_assoc($result);

// Fetch result row. Columns are given keys starting from 0, and indexed onwards
	mysqli_fetch_row($result);

// Fetch result row, giving the data BOTH numeric and string keys
// Combines the features of mysqli_fetch_assoc() and mysqli_fetch_row()
	mysqli_fetch_array($result);

// Return the number of rows in a result set (the number of entries)
	mysqli_num_rows($result);

// Return the id (generated with AUTO_INCREMENT) used in the last query
	mysqli_insert_id($cxn)
```



### 3.3 Querying the Database <a name="3.3"></a> 

[go to top](#top)

```php
ALTER TABLE table change # More info later
CREATE DATABASE database # Self explanatory
CREATE TABLE (col def,…,PRIMARY KEY(col)) # See MySQL tutorial for definition of PRIMARY KEYs
DELETE FROM tablename WHERE clause
DROP database|table
INSERT INTO table (col,col,…) VALUES (col,col,…)
LOAD DATA INFILE ″filename″ INTO TABLE table
SELECT col,col,… FROM table WHERE clause # You SELECT, so that you can display from $result
SELECT statement UNION SELECT statement
SHOW DATABASES|TABLES
SHOW COLUMNS FROM table
UPDATE table SET col=value,… WHERE clause

// Example use
	mysqli_query($dbc, "<INSERT QUERY HERE>");

// Example SELECT query and subsequent printing of data
	$sql = "SELECT id, name FROM MyGuests";
	$result = mysqli_query($conn, $sql); # Now you can print this!

	if (mysqli_num_rows($result) > 0) {
		// output data of each row
		while($row = mysqli_fetch_assoc($result)) { 
		# The while loop works because mysqli_fetch_assoc() iterates until there are no more rows
			echo "id: " . $row["id"]. " - Name: " . $row["name"] . "<br>";
		}
	} else {
		echo "0 results";
	}
```



### 3.4 Altering the Database <a name="3.4"></a> 

[go to top](#top)

**Use these as extra parameters/arguments when querying the database**

More info: http://www.createafreewebsite.net/phpmysql/alter.html

```php
ADD colname definition # ADD a column, name it, and specify its datatype
ALTER colname SET DEFAULT value
ALTER colname DROP DEFAULT
CHANGE colname newcolname definition
DROP colname # REMOVE the column
MODIFY colname definition
RENAME newtablename
  
// Example use
	mysqli_query($dbc,"ALTER TABLE birthdays ADD street CHAR(30) AFTER birthday");
  
```



### 3.5 Small note on -> <a name="3.5"></a> 

[go to top](#top)

```php
// When looking at some PHP files written by other people, you might come across the -> operator
// It stands for accessing the methods or properties of an object in the OOP version of PHP

// Create a new instance of MyObject into $obj
	$obj = new MyDragon();
// Set a property in the $obj object called thisProperty
	$obj->thisProperty = 'methylDragon';
// Call a method of the $obj object named getProperty
	$obj->getProperty();
```



## 4. Reference Links <a name="4"></a>

https://www.youtube.com/watch?v=7TF00hJI78Y (Heavily adapted)  
https://www.cheatography.com/guslong/cheat-sheets/php-syntax-for-beginners/ (Oh look a cheatsheet for PHP!)  
http://www.dummies.com/programming/php/php-mysql-for-dummies-cheat-sheet/ (PHP-MySQL cheatsheet)  
http://www.createafreewebsite.net/phpmysql/alter.html (Alter cheatsheet)    
    

------

[![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)