# Object-Oriented PHP Syntax Reference

Author: methylDragon  
Contains a syntax reference for Object-Oriented PHP  
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=5YaF8xTmxs4

---

## Pre-Requisites

### Assumed knowledge (This is a syntax reference, not a basic coding tutorial)

- How **variables, loops, conditionals, etc**. work
- Basic **HTML**
- Completed the **PHP** syntax tutorial



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
   1.1  [Objects and Classes](#1.1)    
   1.2  [Inheritance](#1.2)    
   1.3  [Polymorphisms](#1.3)    
   1.4  [Interfaces](#1.4)    
2. [OOP Syntax Reference](#2)    
   2.1   [Visibility (Access Levels)](#2.1)    
   2.2   [Class Declaration](#2.2)    
   2.3   [Child Class Declaration](#2.3)    
   2.4   [Object Construction](#2.4)    
   2.5   [Method Polymorphisms via Child Classes](#2.5)    
   2.6   [A note on :: and ->](#2.6)    
   2.7   [A note on self and $this](#2.7)    
   2.8   [Interface Declaration](#2.8)    
   2.9   [Method Polymorphisms via Interfaces](#2.9)    
   2.10 [Some more useful functions](#2.10)    
   2.11 [Abstract Classes and Methods](#2.11)    
3. [Reference Links](#3)  



## 1. Introduction <a name="1"></a>

Let's do a refresher for some basic Object Oriented Programming (OOP) concepts! 

> The basis for all object oriented programming languages is the manipulation of **objects**.
>
> **Objects** contain **Attributes** and **Methods**. These two can be referred to collectively as the **Members** of the **Object's Class**.



### 1.1 Objects and Classes <a name="1.1"></a>

[go to top](#top)

**Objects** act a lot like real-world objects. They:

- Can be **created** or **destroyed**
- Are instances of **Classes**
- Have manipulable **Properties/Attributes**
- Have callable **Methods** (it's like calling functions!)

**Classes** define the **Attributes** and **Methods** of objects that belong to them! Attributes and Methods are referred collectively as the class' **Members**.

So for example, you can have a class that defines an object with the property: "Colour", but the instance of the class (the object), can have any value within that property (Red, Black, etc.) **You can also state the default value of the class otherwise!**

> **Example:** **methylDragon** is an object that belongs to the **Dragon** class. This gives him default attributes like:
>
> Color: Red
> Sound: ["Rawr"]  (This is an array because the object might make multiple sounds)
> Breath: Fire
>
> And methods like:
>
> Sound()
> Talk()
> Walk()
> Eat()
> Fly()
>
> **Note:** When constructing an object belonging to a class, you **can** override the default values defined by the class. So when defining **methylDragon**, you can give him alternative attribute values! Like the **Color: Black**!



### 1.2 Inheritance <a name="1.2"></a>

[go to top](#top)

Notably, **you can nest classes!** Sub-classes will then **inherit** or **overriding** the attributes and methods of their parent classes as a **polymorphism**.

> **Example:** The class **Dragon** has a child class (sub-class) **Music_Dragon** that **methylDragon** belongs to! 
>
> This can confer new methods or attributes (Like... A stage-name, and music making ability) **in addition** to any attributes and methods the **Dragon** class conferred.
>
> It can also override any pre-existing methods or attributes the defined objects would have had. So for example, Sound could be re-defined as containing ["Rawr", "~♪~♫~♪~♪"] by default instead!
>
> So now **methylDragon** has the following:
>
> Attributes:
>
> Color: Black | (<u>Custom defined</u>)
> Sound: ["Rawr", "~♪~♫~♪~♪"] | (<u>Overridden</u>)
> Breath: Fire | (<u>Inherited</u>)
>
> Methods:
>
> Sound() | (<u>Overridden</u>)
> Talk() | (<u>Inherited</u>)
> Walk() | (<u>Inherited</u>)
> Eat() | (<u>Inherited</u>)
> Fly() | (<u>Inherited</u>)



### 1.3 Polymorphisms <a name="1.3"></a>

[go to top](#top)

You can override pre-existing methods defined by a parent class. This means that when the method is called, the child class' method definition will be used instead of the parent class'.

> **Example:** The **Music_Dragon** class defines an alternate version of the Sound() method that allows objects belonging it to also make music!
>
> So **methylDragon** can make music! :musical_note:



### 1.4 Interfaces <a name="1.4"></a>

[go to top](#top)

http://php.net/manual/en/language.oop5.interfaces.php

> Object interfaces allow you to create code which specifies which methods a class must implement, without having to define how these methods are handled.  
>
> Interfaces are defined in the same way as a class, but with the *interface* keyword replacing the *class* keyword and without any of the methods having their contents defined.  
>
> All methods declared in an interface must be public; this is the nature of an interface.

> Imagine that I make cars, and you drive them.
>
> Me, as the car manufacturer, don't want you to be messing around with it's internals (motor, ECU, etc), but I do want you to drive it.
>
> So I create the interface, wheel, buttons, etc. So you just need to learn the interface. However, nothing stops you to pop up the hood and see what's in it, right?
>
> Today I manufacture cars, tomorrow it can be a van, but I'll keep the same interface, and maybe add some options, so that you can still drive it without learning every little detail about cars, vans, trucks, motorcycles and so on. (r/eli5)

In short, implementing an interface is like having an object sign a contract to implement any method defined in the interface.



## 2. OOP Syntax Reference <a name="2"></a>

### 2.1 Visibility (Access Levels) <a name="2.1"></a>

[go to top](#top)

You can restrict the visibility of an object's methods and attributes for debugging, and security purposes!

The most commonly used visibility levels are: **`public`, `private`**, and **`protected`**.

**If you don't declare any visibility levels, the default is public. But it's good to be explicit about it.**

```php
public:
// Public methods and attributes can be accessed and changed by
# any class

private:
// Private methods and attributes can only be accessed and changed by
# the same class that declared it

protected:
// Protected methods and attributes can be accessed and changed by
# the same class that declared it
# any child classes (that inherit from the class the protected data is declared in)

# NOTE: Visibility affects inheritance as well! Private attributes and methods are NOT inherited
```



### 2.2 Class Declaration <a name="2.2"></a>

[go to top](#top)

**Let's define an Animal class**

```php
class Animal {
```
**Now we can start defining attributes**

```php
	// In this case, there are no default values
	protected $name;
	protected $sound;
	protected $id;
  
	public static $number_of_animals = 0;
	# Static means that every object of class Animal will share this attribute and its value. 
	# If we increment this, every object of class Animal will reflect the change. Its default
	# value is 0.

	// You can also define constants! ... If you want to.. For some reason.
	# You can access this particular one using Animal::e
	const e = "2.71828";
```

**Now we can define methods!**

```php
	// Animals must be able to move! Let's make a function for that!
	public function move() {
		echo $this->name . " runs<br/>";
		// $this refers to the object the method is called from
		// $this->name: "Access the data stored in $name from $this object"
    }
```
**And a `final` method**
```php
// Let's define a method that allows us to access the private attribute $name
// Now you can access it by calling it from any Animal object!
# The "final" keyword prevents child classes from overriding them!
	final public function getName() {
		return $this->name;
    }
```
**And a `static` method**

```php
// Static methods are defined using the "static" keyword
# Static methods can be called without the need to instantiate/create an object
# You call them using ClassName::method()
	static function add_these($num1, $num2) {
		return ($num1 + $num2) . "<br/>";
	}
```

**We're defining constructors and destructors here**

```php  
// We can also define special methods with __
# They're called Magic Methods! Google it!
# The double underscores __ denote that the method is a special method. There are others!

	// Now let's write a constructor method!
  	// Class constructor methods are called each time an object/instance of the class is created
	public function __construct() {
		$this->id = $number_of_animals + 1; // Set the instance's $id

		echo $this->id . " has been assigned<br/>"; // Echo out a notice
      
		// Now let's increment the static attribute $number_of_animals
		# You can access this using Animal::$number_of_animals
		Animal::$number_of_animals++; // We're incrementing it once here
	}

	// Destructor methods called each time an object/instance of the class is destroyed/unset
	public function __destruct() {
		echo $this->name . " is being destroyed<br/>" // Echo out a notice
	}
```
**Magic methods**

```php
// PHP has some magic methods that allow us to create Getters and Setters easily
# Note: These are slower than using direct method calls

	// Getters are functions that access and get inaccessible data
	public function __get($attribute) {
		echo "Asked for " . $attribute . "<br/>";
		return $this->$attribute;
	}

	// Setters are methods that set inaccessible data
	public function __set($attribute, $value) {
		switch($attribute) {
			case "name" :
				$this->name = $value;
				break;
			case "sound" :
				$this->sound = $value;
				break;
			default :
				echo $attribute . "Not Found";
				die();
		}
		echo "Set " . $attribute . " to " . $value . "<br/>";
	}

	// The __toString() method runs each time a class instance is echoed and prints whatever is returned
	// example: echo $object;
	public function __toString() {
  		return $this->name . " is an Animal that says " . $this->sound;
	}
}
```



### 2.3 Child Class Declaration <a name="2.3"></a>

[go to top](#top)

**Now let's make a sub-class!** Remember that child classes inherit every attribute and method **except for private ones!**

Let's just remake the Dragon class. (Forget anything you knew about the class from the introduction section.)

```php
// The "extends" keyword is how you declare a child class
class Dragon extends Animal {
	// Dragons fly! So let's override the move() method
  	# Note: You cannot ovveride methods declared as "final"
	public function move() {
		echo $this-> name . " flies<br/>";
	}
}
```



### 2.4 Object Construction <a name="2.4"></a>

[go to top](#top)

Now let's make some objects!

**Let's create an Animal called Smaug**

```php
// Create new instances of classes (objects) using the "new" keyword!
	$animal_one = new Animal();

// Set attributes by accessing them using ->
	$animal_one->name = "Smaug";
	$animal_one->sound = "I am Fire. I am Death.";

// Now you can use these attributes as variables!
	echo $animal_one->name . " says " . $animal_one->sound . "<br/>";
	echo $animal_one->name . "'s ID is " . $animal_one->id . "<br/>";
	echo "Total Animals: " . Animal::$number_of_animals . "<br/><br/>";

// Get the class name of an object using get_class()
	echo $animal_one->name . " is a/an " . get_class($animal_one) . ".<br/>";
	# This will echo "Smaug is a/an Animal."

// And if you remember, we defined a static function
	echo "3 + 5 = " . Animal::add_these(3,5);
	# This will print "3 + 5 = 8"
```



### 2.5 Method Polymorphisms via Child Classes <a name="2.5"></a>

[go to top](#top)

**Now let's construct a Dragon called methylDragon** (because of course)

Let's see how calling move() differs between `$Smaug` and `$methylDragon`

```php
$toothless = new Dragon();

$toothless->name = "Toothless";
$toothless->sound = "Raa"
  
// Call an object's class method using $object->method()
	$toothless->move();
	// This will result in "Toothless flies" being printed
	$animal_one->move();
	// This will result in "Smaug runs" being printed

# The polymorphism worked!
```



### 2.6 A note on `::` and `->` <a name="2.6"></a>

[go to top](#top)

https://stackoverflow.com/questions/1245121/difference-between-and-in-php-mysqli-oop
https://stackoverflow.com/questions/7524503/php-difference-between-and

> Use `->` to access **static or non-static** members of an **object**

> Use `::` to access **ONLY static** members of an **object** or **class**



### 2.7 A note on `self` and `$this` <a name="2.7"></a>

[go to top](#top)

https://stackoverflow.com/questions/151969/when-to-use-self-over-this

> Use `$this` to refer to the **current object.** 
> In other words, use `$this->member` for non-static members

> Use `self` to refer to the **current class.** 
> In other words, use `self::$member for static members

```php
// Here's a correct implementation!
class myClass {
    private $non_static_member = 1;
    private static $static_member = 2;

    function __construct() {
        echo 'Non-Static: ' . $this->non_static_member . ' Static: ' . self::$static_member;
    }
}
```



### 2.8 Interface Declaration <a name="2.8"></a>

[go to top](#top)

```php
// Declare interfaces using the "interface" keyword
interface Singable {
	public function sing();
}

# Note: You don't actually say what the method sing() does here. Interfaces are a way for you to help with method polymorphisms!
# Interfaces are better than child classes because you can only "extend" an object once, but you can append a neverending array of interfaces.
```

**NOTE: Interfaces** will **force** you to define the required methods within the classes you implement them in!

**Let's implement the Singable interface now!**

```php
// You let a class implement an interface using the "implements" keyword
class Music_Dragon extends Dragon implements Singable {
	function sing() {
		echo $this->name . " sings '~♪~♫~♪~♪'<br/>";
    }
}

class Human extends Animal implements Singable {
	function sing() {
		echo $this->name . " sings 'Ohhh~♪~♫'<br/>"; 	
	}
}
```



### 2.9 Method Polymorphisms via Interfaces <a name="2.9"></a>

[go to top](#top)

You can define functions that accept classes that **extend** a certain class or implements a certain **interface**.

```php
// Let's make some objects now
	$methylDragon = new Music_Dragon();
	$bob = new Human();

// And we'll give them some names
	$methylDragon->name = "methylDragon";
	$bob->name = "Bob the human";

// Let's make a function that accepts classes that implement Singable
	function make_them_sing(Singable $singing_animal) {
		$singing_animal->sing();
}

// Now we'll call them!
	make_them_sing($methylDragon);
// Prints "methylDragon sings '~♪~♫~♪~♪'"
	make_them_sing($bob);
// Prints "Bob the human sings 'Ohhh~♪~♫'"
```



### 2.10 Some more useful functions <a name="2.10"></a>

[go to top](#top)

```php
// Ternary operator refresher: ($var > 2) ? true : false
// "Is $var greater than 2? If yes, return true, if no, return false

// You can check if an object is an instance of a class using "instanceof"
	$is_it_an_animal = ($methylDragon instanceof Animal) ? "True" : "False";
	echo "It is " . $is_it_an_animal . " that $methylDragon is an Animal<br/>"
	# In this case, this will echo "It is True that $methylDragon is an Animal"
	# Because remember, $methylDragon is a Music_Dragon, which is has Dragon, and hence Animal as a parent class
      
// Clone objects using the "clone" keyword!
	$smaug_clone = clone $animal_one;
	# The magic method __clone() will be called everytime a object is cloned

```



### 2.11 Abstract Classes and Methods <a name="2.11"></a> 

[go to top](#top)

Abstract classes cannot be instantiated, but it **forces** classes that implement it to **override** any abstract methods that are members of the abstract class.

```php
// Define an abstract class or method using the "abstract" keyword
	abstract class RandomClass {
		abstract function RandomFunction($attribute);
	}
```



## 3. Reference Links <a name="3"></a>

https://www.youtube.com/watch?v=5YaF8xTmxs4 (Heavily adapted)  
http://www.logicbig.com/tutorials/misc/php/php-oop-cheat-sheet/ (Oh look! A more in-depth cheat sheet!)  

​    

------

 [![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)