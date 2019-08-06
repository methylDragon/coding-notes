# Advanced C++ Crash Course (OOP)

Author: methylDragon  
Contains an advanced syntax reference for C++  
This time, we'll be going through OOP!   
I'll be adapting it from the ever amazing Derek Banas: https://www.youtube.com/watch?v=Rub-JsjMhWY

------

## Pre-Requisites

**Assumed knowledge (This is a C++ crash course, not a basic coding tutorial)**

- How **variables, loops, conditionals, etc**. work (Basic coding fundamentals will help a lot!)
- Linux (**Terminal/Console proficiency**) (We're going to need to compile our stuff)
- Gone through the all preceding parts of the tutorial



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
   1.1  [Objects and Classes](#1.1)    
   1.2  [Inheritance](#1.2)    
   1.3  [Polymorphisms](#1.3)    
   1.4  [Interfaces](#1.4)    
2. [OOP C++ Syntax Reference](#2)    
   2.1   [Visibility](#2.1)    
   2.2   [Access Operators](#2.2)    
   2.3   [Classes](#2.3)    
   2.4   [Child Class Declaration](#2.4)    
   2.5   [Virtual Methods and Polymorphisms](#2.5)    
   2.6   [Friendship](#2.6)    
   2.7   [Operator Overloading](#2.7)    
   2.8   [Rules for Object Management](#2.8)    



## 1. Introduction <a name="1"></a>

Object-Oriented Programming. Pretty standard, some small quirks in C++, but good vibes. :)

> The basis for all object oriented programming languages is the manipulation of **objects**.
>
> **Objects** contain **Attributes** and **Methods**. These two can be referred to collectively as the **Members** of the **Object's Class**.



### 1.1 Objects and Classes <a name="1.1"></a>

[go to top](#top)

> These subsections are pulled from my PHP OOP reference, but I'm including them here for convenience's sake 

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
> So **methylDragon** can make music! 🎵



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



## 2. Object-Oriented C++ Syntax Reference <a name="2"></a>

### 2.1 Visibility <a name="2.1"></a>

[go to top](#top)

Let's get some pre-requisites refreshed or out of the way.

If this tutorial's going too fast for you, go brush up on some OOP concepts. I treated the subject fairly well in the Object-Oriented PHP reference.

```c++
// Visibility refresher

public:
// Public methods and attributes can be accessed and changed by:
// - any class

private: // DEFAULT!
// Private methods and attributes can only be accessed and changed by:
// - the same class that declared it

protected:
// Protected methods and attributes can be accessed and changed by:
// - the same class that declared it
// - any child classes (that inherit from the class the protected data is declared in)

// NOTE: Visibility affects inheritance as well! Private attributes and methods are NOT inherited
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
#include <iostream>

using namespace std;

class Car {
    public:
    	int number;
    
    	void Create() {
			cout << "Car created, number is: " << number << "\n" ; 
        }
}

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
	(*y).Create(); // *y is the object
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

class Animal {
	private:
  		int height;
  		int weight;
  		string name;
  
  		static int numOfAnimals; // Static variables are shared by every object of the class
  		// Static variables are normally attributes that the class object normally wouldn't have
  		// They are usually properties of groups, not individuals!
  
  	public: // The way this has been done is called encapsulation! It increases security.
  		
    	// Getter methods
    	int getHeight(){ return height; }
  		int getWeight(){ return weight; }
  		string getName(){ return name; }
  		
    	// Setter methods
    	void setHeight(int cm){ height = cm; } 
  		// You can use a conditional here to keep things sensible
  		void setWeight(int kg){ height = kg; }
  		void setName (string animalName){ name = animalName; }
  
  		void setAll(int, int, string);
  
  		// This is our constructor! Constructors are named the same name as the class
    	Animal(int, int, string); 
    	Animal(); // Or an overloaded function call
  		
    	// Destructor here
  		~Animal();
  
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
    // Note, we only need to use this -> if the variable names clash
    // If the constructor had (int i_height) for example, then it's
    // totally fine to do height = i_height
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



#### **A better way to initialise (initialisation lists)**

Remember how we wrote the constructor?

```c++
Animal::Animal(int height, int weight, string name){
	this -> height = height;
	this -> weight = weight;
	this -> name = name;
	Animal::numOfAnimals++;
}
```

Remember **direct initialisation** in part 1 of this crash course?

No? Just recall that it's more efficient than assignment, since assignment createanother object in memory. So it's generally always better to do direct initialisation.

How do we do it for multiple variables at a time? Use **initialisation lists**

```c++
Animal::Animal(int init_height, int init_weight, string init_name)
    // Here's the init list! It's a little long, so we put it on the next line, indented
    : height(init_height), weight(init_weight), name(init_name)
    {
        // Aaand then whatever other code we want to run!
        // (Since the constructor IS a function)
    }
        
        
    // Note: We didn't use this -> since the argument names were changed to init_<foo>
```

So just remember that the next time you see a random `:` where it doesn't seem like there should be one!



#### **Structs** (Data Structures)

Read more: http://www.cplusplus.com/doc/tutorial/structures/

They're just like classes! Except that their default member visibility is public.

That means you can also write a constructor into them!

```c++
struct my_struct {
    int a;
    auto b;
    
    // Constructor with member initialisation
    my_struct(int a_, auto b_)
        : a(a_), b(b_) {}
};

// Make the structure object
my_struct structure(1, 2);

// Change its properties
structure.a = 2
```



### 2.4 Child Class Declaration <a name="2.4"></a>

[go to top](#top)

Note: https://stackoverflow.com/questions/860339/difference-between-private-public-and-protected-inheritance

> ```
> Member in base class :   Private   Protected   Public   
> ```
>
> ​        **Inheritance type**              :       **Object inherited as**
>
> ```
> Private              :   Inaccessible   Private     Private   
> Protected            :   Inaccessible   Protected   Protected  
> Public               :   Inaccessible   Protected   Public
> ```

```c++
// Let's make a new class that inherits from Animal!

// So this is saying it inherits everything inheritable from Animal, and those inherited members become public

// If, however, it was class Dragon : private Animal
// Then all the inherited members become private for Dragons
class Dragon : public Animal { 
  
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

> Note: It is possible for a class to be a child class of multiple classes! This is called **multiple inheritance.**
>
> Just state it as such
>
> ```c++
> class Child : public Parent_1, public Parent_2{
> 
> // so on and so forth
> 
> };
> ```
>



### 2.5 Virtual Methods and Polymorphisms <a name="2.5"></a>

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
  Animal *animal = new Animal; // The 'new' keyword generates a pointer (a memory address
  Dragon *dragon = new Dragon; // to be stored)
    
  // Using new is just an option, it's not recommended unless you absolutely have to
  // In this case you don't (just change the whatClassAreYou function arguments)
  // But I just wanted to show it
    
  // https://stackoverflow.com/questions/655065/when-should-i-use-the-new-keyword-in-c
  
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



### 2.6 Friendship <a name="2.6"></a>

[go to top](#top)

Read more: http://www.cplusplus.com/doc/tutorial/inheritance/

Yes yes. It's a cute name. Let's move on.

**Friends** of a class (be they **classes** or **functions**) are able to access the private and protected members of a class even if they are outside the class or the child classes!

You do this by using the `friend` keyword.

```c++
// Let's do an example

class Dragon {
    // The default visibility is private
    // So there's technically no need to state private:
    int happiness;
    int hunger;
    
  public:
    // "Friendly"
    friend int friendly_dragon_happiness_detector(const Dragon&);

    // Here's our constructor
    Dragon(int init_happiness, int init_hunger)
        : happiness(init_happiness), hunger(init_hunger) {}
    
  // Some people say kobolds have an indepth insight into the state of their dragon
  // I don't have any kobolds though. Kind of a stereotype.
  friend class Kobold; // Kobolds are (allegedly) friends of dragons!
}

// Here we list the dragon friends!

class Kobold {
    // Some implementation
}

int friendly_dragon_happiness_detector(const Dragon& dragon){
    return dragon.happiness // Boom. It'll work because it's 'friendly'
    // But the folks really need to respect dragon privacy. Tsktsk.
}
```



### 2.7 Operator Overloading <a name="2.7"></a>

[go to top](#top)

You can redefine what an operator does to a class!

So you can change what the `+` operator does, for example.

All you need to do, is use the `operator` keyword, followed by the operator that you want to redefine. (Do this inside a class, of course.)

```c++
public:
  MyClass operator + (MyClass const &MyClassObj)
  {
    // Do some stuff
    return something;
  }
```

You can also overload conversion operators!

```c++
class Wow
{
public:
  Wow() {} // Some contructor

  operator float()
  {
    // Do some float calculation stuff
    return float_conversion_result;
  }    
}

// Our new overloaded float conversion operator will be usable like so!
a = Wow();
float b = a; // Wow!
```

**Example**

```c++
// Source: https://www.geeksforgeeks.org/operator-overloading-c/

#include<iostream> 
using namespace std; 
  
class Complex { 
private: 
    int real, imag; 
public: 
    Complex(int r = 0, int i =0)  {real = r;   imag = i;} 
      
    // This is automatically called when '+' is used with 
    // between two Complex objects 
    Complex operator + (Complex const &obj) { 
         Complex res;
         res.real = real + obj.real; 
         res.imag = imag + obj.imag; 
         return res; 
    } 
    void print() { cout << real << " + i" << imag << endl; } 
}; 
  
int main() 
{ 
    Complex c1(10, 5), c2(2, 4); 
    Complex c3 = c1 + c2; // An example call to "operator+" 
    c3.print(); 
} 
```

#### **Caveats**

Almost every operator can be overloaded, except for...

- `.`
- `::`
- `?:`
- `sizeof`

[Reasons being...](<http://www.stroustrup.com/bs_faq2.html#overload-dot>)



### 2.8 Rules for Object Management <a name="2.8"></a>

[go to top](#top)

#### **Introduction**

C++ provides 6 special functions that are called default operations that deal with managing an object's lifecycle.

|      Operation      |       Signature       |
| :-----------------: | :-------------------: |
| Default constructor |         `X()`         |
|  Copy constructor   |     `X(const X&)`     |
|   Copy assignment   | `operator=(const X&)` |
|  Move constructor   |       `X(X&&)`        |
|   Move assignment   |   `operator=(X&&)`    |
|     Destructor      |        `~X()`         |

You can explicitly define them, delete them, or lean on the compiler's default.

```c++
X() {}; // Default constructor, explicitly defined
X() = default; // Default constructor, defaulting to compiler implementation
X() = delete; // Explicitly deleted default constructor. There will be no implementation
```

The rules for object management help to keep your programs running as expected. You can treat the rules like boilerplate, but just keep them in mind.



#### **Rule of Zero**

This one is fairly simple. Since it keeps things simple.

>  **If you can avoid redefining any default operations, do.**



#### **Rule of Three**

> **If you redefine at least one of the following operations: Destructor, Copy Constructor, Copy Assignment, explicitly redefine all three.**

Example:

- Suppose we have a class with a redefined destructor `~X()`
- When the default copy assignment `operator=(const X&)` or copy constructor `X(const X&)` is used, it copies all class members, including the redefined destructor, and **any declared local pointers without reallocating extra memory for those copied local pointers**.
  - In other words, using the default copy assignment and copy constructor operations, all pointers local to each class instance point to the same piece of memory!
- So when the program goes out of scope, for the same destructor is called for all copies of the object, the same address pointed to by all pointers in instances of the class are deleted multiple times, crashing the program.

Similar logic can be used for the other two operations. If you only define one copy operation, the other gets used as the default and will really mess up your memory management.



#### **Rule of Five**

Actually all six operations are intimately linked. So the rule of five (it's the rule of six if you count the default constructor) exists to remind you that...

> **If you redefine or =delete any default operation, define or =delete them all.**

If you don't follow the rule, your objects might behave weirdly.

Furthermore, when you actually define them, make sure you use **consistent semantics**. (Example: If you use shallow copies (copying of pointers) in a single copy operation, do not use deep copies (copying pointers and pointed data) for the other copy operator))

(You'll generally want to use deep copies usually anyway...)

```c++
// Shallow copy
p = a.p;

// Deep copy
p= new int(*(a.p))
```



```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

​        

---

 [![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)