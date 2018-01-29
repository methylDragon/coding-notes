# OOP Python 3 Crash Course

Author: methylDragon  
Contains an advanced syntax reference for Python 3  
This time, we'll be going through OOP!   
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
2. [OOP C++ Syntax Reference](#2)    
   2.1   [Visibility](#2.1)    
   2.2   [Access Operators](#2.2)    
   2.3   [Classes](#2.3)    
   2.4   [Child Class Declaration](#2.4)    
   2.5   [Virtual Methods and Polymorphism](#2.5)    




## 1. Introduction <a name="1"></a>

Object-Oriented Programming. Pretty standard, good vibes. :)

Why Object-Oriented though? It's good to model programming structures like how real world objects are like! Real world objects have properties (colour, height, etc.) and abilities (run, walk, swim)! 

Python 3 objects have Properties and Methods! These are collectively known as the Elements of an object!

## 2. Object-Oriented Python 3 Syntax Reference <a name="2"></a>

### 2.1 Visibility <a name="2.1"></a>

Let's get some pre-requisites refreshed or out of the way.

If this tutorial's going too fast for you, go brush up on some OOP concepts. I treated the subject fairly well in the Object-Oriented PHP reference.

```python
# Visibility refresher

public:
# Public methods and attributes can be accessed and changed by:
# - any class

private:
# Private methods and attributes can only be accessed and changed by:
# - the same class that declared it

protected:
# Protected methods and attributes can be accessed and changed by:
# - the same class that declared it
# - any child classes (that inherit from the class the protected data is declared in)

# NOTE: Visibility affects inheritance as well! Private attributes and methods are NOT inherited
```

In Python 3,

- Public elements are defined by default. Just write the element name and you've set it as public! `element_name`
- Private elements are defined by preceding the element name with two underscores `__element_name`
- Protected elements are defined by preceding the element name with one underscore `_element_name`

### 2.2 Class Definition <a name="2.2"></a>

[go to top](#top)

Let's define a Class! Class objects are templates that objects are instances of.

So for example, you can have an Animal class which defines common properties and methods that every animal should have. But an individual animal will just be an instance of this class (that can have different values in those properties!)

```python
class Animal:
    # We're defining properties here! Think of them as class specific variables!
    # These are the default values that instances of the class (objects) are initiated with
    __name = ""
    __height = 0
    __weight = 0
    __sound = 0
    
    # We're defining methods here! Think of them as class specific functions!
    def set_name(self, name): # Self refers to the instance of the class when the method is called!
        self.__name = name
```













### 2.2 Access Operators <a name="2.2"></a>

[go to top](#top)

https://stackoverflow.com/questions/4984600/when-do-i-use-a-dot-arrow-or-double-colon-to-refer-to-members-of-a-class-in-c

> The three distinct operators C++ uses to access the members of a class or class object, namely the double colon `::`, the dot `.`, and the arrow `->`, are used for three different scenarios that are always well-defined. Knowing this allows you to immediately know quite a lot about `a` and `b` just by looking at `a::b`, `a.b`, or `a->b`, respectively, in any code you look at. 
>
> 1. `a::b` is only used if `b` is a member of the class (or namespace) `a`. That is, in this case `a` will always be the name of a class (or namespace). 
> 2. `a.b` is only used if `b` is a member of the object (or reference to an object) `a`. So for `a.b`, `a` will always be an actual object (or a reference to an object) of a class. 
> 3. `a->b` is, originally, a shorthand notation for `(*a).b`. However, `->` is the only of the member access operators that can be overloaded, so if `a` is an object of a class that overloads `operator->` (common such types are smart pointers and iterators), then the meaning is whatever the class designer implemented. To conclude: With `a->b`, if `a` is a pointer, `b` will be a member of the object the pointer `a` refers to. If, however, `a` is an object of a class that overloads this operator, then the overloaded operator function `operator->()` gets invoked. 

Basically,

`::` Is used to access static attributes and methods (it's called the scope operator)

`.` Is used to access methods belonging to the object (private, public, protected) "Access"

`->` Is another way to do this. But we normally use it to specify change values stored in an object via passing by reference. "Access"

Illustrative Example:

```c++
// The two chunks for (x) and (*y) are equal
// Credit: https://www.programcreek.com/2011/01/an-example-of-c-dot-and-arrow-usage/

int main() {
	Car x;
	// declares x to be a Car object value,
	// initialized using the default constructor
	// this very different with Java syntax Car x = new Car();
	x.number = 123;
	x.Create();
 
	Car *y; // declare y as a pointer which points to a Car object
	y = &x; // assign x's address to the pointer y
	(*y).Create(); // *y is object
	y->Create(); // same as previous line, y points to x object. It stores a reference(memory address) to x object.
 
	y->number = 456; // this is equal to (*y).number = 456;
	y->Create();
}
```



### 2.3 Classes <a name="2.3"></a>

[go to top](#top)

Refresher: In Object-Oriented Programming (OOP), the goal is to model objects that exist in the real world with code structures!

>  OOP concepts are going to be assumed in this reference!!!

Objects then, have:

- Properties/Attributes
- Abilities/Methods

Objects can also be categorised! So in OOP, objects can be created as instances (independent copies) of Classes!

```c++
// Example class definition
// Functions that haven't been defined will be prototyped somewhere else

using namespace std;

class Animal{
	private:
  		int height;
  		int weight;
  		string name;
  
  		static int numOfAnimals; // Static variables are shared by every object of the class
  		// Static variables are normally attributes that the class object normally wouldn't have
  		// They are usually properties of groups, not individuals!
  
  	public: // The way this has been done is called encapsulation! It increases security.
  		int getHeight(){ return height; }
  		int getWeight(){ return weight; }
  		string getName(){ return name; }
  		void setHeight(int cm){ height = cm; } 
  		// You can use a conditional here to keep things sensible
  		void setWeight(int kg){ height = kg; }
  		void setName (string animalName){ name = animalName; }
  
  		void setAll(int, int, string);
  
  		Animal(int, int, string); 
  		// This is our constructor! Constructors are named the same name as the class
  		Animal();
  		// Overloading example here
  		~Animal();
		// Destructor here!
  
  		static int getNumOfAnimals() { return numOfAnimals; }
		// Static methods are attached to classes and not objects
		// They can only access static variables!
  
  		void toString();
}; // This semi-colon is required!
```

```c++
// Aaaand we're defining our function prototypes here

int Animal::numOfAnimals = 0;

void Animal::setAll(int height, int weight, string name){
	this -> height = height;
	this -> weight = weight;
	this -> name = name;
	Animal::numOfAnimals++;
}

Animal::Animal(int height, int weight, string name){
	this -> height = height;
	this -> weight = weight;
	this -> name = name;
	Animal::numOfAnimals++;
}

Animal::~Animal(){
	cout << "Animal " << this -> name << " destroyed" << endl;
}

Animal::Animal(){
	Animal::numOfAnimals++;
}

void Animal::toString(){
	cout << this -> name << " is " << this -> height << " cm tall and " << this -> weight << " kg in weight" << endl;
}
```

```c++
// Let's make some objects!!!!

int main(){
	Animal fred;
  	fred.setHeight(33);
  	fred.setWeight(10);
  	fred.setName("Fred");
    
  	Animal tom(36, 15, "Tom");  	
}

// Welp, we made them
```



### 2.4 Child Class Declaration <a name="2.4"></a>

[go to top](#top)

Note: https://stackoverflow.com/questions/860339/difference-between-private-public-and-protected-inheritance

> ```
> Member in base class : Private   Protected   Public   
> ```
>
> **Inheritance type**  :             **Object inherited as**:
>
> ```
> Private            :   Inaccessible   Private     Private   
> Protected          :   Inaccessible   Protected   Protected  
> Public             :   Inaccessible   Protected   Public
> ```

```c++
// Let's make a new class that inherits from Animal!

class Dragon : public Animal { // So this is saying it inherits everything inheritable from Animal
  
	private:
  		string sound = "Rawr"; // Heh. I mean. Rawr.
  
  	public:
  		void getSound() { cout << sound << endl; }
  		Dragon(int, int, string, string); // Here's our constructor
  		Dragon() : Animal(){}; // Here's an overloaded one. With a superclass constructor!

  		void toString();
};
```

```c++
// Aaaand our function prototypes

Dragon::Dragon(int height, int weight, string name, string roar) : 
Animal(height, weight, name){ 
// The Dragon constructor inherits the function prototype for the Animal constructor
  	this -> sound = roar; // But now can handle sound IN ADDITION to whatever it had from Animal!
}

void Dragon::toString(){
  	cout << this -> getName() << " is " << this -> getHeight() << " cm tall and " << this -> getWeight() << " kg in weight and says " << this -> sound << endl;
  
  	// Notice we had to use this -> <method>. This is because the height, weight, and name
  	// attributes from the base class are private and can't be accessed by the inheriting class!
}
```

```c++
// Let's make a dragon! Exciting!

Dragon methylDragon(201, 89, "methylDragon", "~♪~♫~♪~♪");

cout << "Number of Animals " << Animal::getNumOfAnimals() << endl;
// If we ran this with the previous section (where we created Animals)
// This would output: Number of Animals 3

spot.getSound();
tom.toString();
// etc. etc. etc.
```



### 2.5 Virtual Methods and Polymorphism <a name="2.5"></a>

[go to top](#top)

Virtual methods are used when you know that the class you're defining will be a base class that may have the method be overridden by a child class!

FYI: You can also create virtual **attributes** that can get overriden in the same way! I didn't write the example here, but it still works!

Polymorphisms are when subclasses implement different methods/functions differently from their base class!

```c++
// Let's just redo everything from the top with the Animal class

using namespace std;

class Animal{
  	public:
  		void getFamily() { cout << "We are animals" << endl; }
  		/*virtual*/ void getClass() { cout << "I'm an Animal" << endl; }
};

class Dragon : public Animal{
  	public: getClass() { cout << "I'm a Dragon!" << endl; } // Polymorphism here!
};

void whatClassAreYou(Animal *animal){
  	animal -> getClass();
};
```

```c++
int main(){
  Animal *animal = new Animal; // The 'new' keyword generates a pointer (a memory address to
  Dragon *dragon = new Dragon; // be stored)
  
  animal->getClass(); // Prints: I'm an Animal
  dragon->getClass(); // Prints: I'm a Dragon!
  
  // This all seems cool and good. But what if we passed objects to the function instead?
  
  whatClassAreYou(animal); // Prints: I'm an Animal   (..ok)
  whatClassAreYou(dragon); // Prints: I'm an Animal   (HOLD ON A SEC)
  // This is happening because dragons are also Animals! How do we fix this?
  
  // Simple! Ensure that the function that might change is specified as virtual!
  // virtual void getClass() {...}
  // This ensures that C++ will check to see if there's a polymorphism
  
  return 0;
}
```



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

