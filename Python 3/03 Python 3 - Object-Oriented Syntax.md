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

1. [[Introduction](#1)  
   1.1  [Objects and Classes](#1.1)    
   1.2  [Inheritance](#1.2)    
   1.3  [Polymorphisms](#1.3)    
2. [OOP C++ Syntax Reference](#2)    
   2.1   [Visibility](#2.1)    
   2.2   [Class Definition](#2.2)    
   2.3   [Child Class Definition](#2.3)    
   2.4   [Polymorphisms](#2.4)    
   2.5   [Magic Methods](#2.5)    
   2.6   [Generating Multiple Objects at a Time](#2.5)   




## 1. Introduction <a name="1"></a>

Object-Oriented Programming. Pretty standard, good vibes. :)

Why Object-Oriented though? It's good to model programming structures like how real world objects are like! Real world objects have properties (colour, height, etc.) and abilities (run, walk, swim)! 

Python 3 objects have Properties and Methods! These are collectively known as the Elements of an object!

> The basis for all object oriented programming languages is the manipulation of **objects**.
>
> **Objects** contain **Attributes** and **Methods**. These two can be referred to collectively as the **Members** of the **Object's Class**.



### 1.1 Objects and Classes <a name="1.1"></a>

[go to top](#top)

>  These subsections are pulled from my PHP OOP reference, but I'm including them here for convenience's sake 

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



### 

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
    _name = ""
    _height = 0
    _weight = 0
    _sound = 0

    # We're defining methods here! Think of them as class specific functions!

    # Define a constructor method that runs each time a new isntance of the class is instantiated
    def __init__(self, name, height, weight, sound):
        # Initialise all the properties as input
        # We're setting these as protected because we want sub-classes to inherit them
        self._name = name
        self._height = height
        self._weight = weight
        self._sound = sound

    # Define setter methods (methods that set object properties)
    def set_name(self, name): # Self refers to the instance of the class when the method is called!
        self._name = name

    # You could go on to define the rest of them for the rest of the properties
    # I won't though

    # Define getter methods (methods that set objectproperties)
    def get_name(self): # Because the name is private
        return self._name
    
    def get_sound(self):
        return self._sound

    # Same here

    # One more method here:
    def get_type(self):
        print("Animal")

    # This method defines what happens when you print the object!
    def __str__(self):
        return "{} is {} cm tall, {} kg and says {}".format(self._name,
                                                              self._height,
                                                              self._weight,
                                                              self._sound)
```

```python
# Cool! Now let's make some objects!
cat = Animal("Cate", 33, 10, "Meow")

print(cat) # Prints: Cate is 33 cm tall and 10 kg and says Meow

# You can call an object's methods like so
print(cat.get_name()) # Prints: Cate
```



### 2.3 Child Class Definition <a name="2.3"></a>

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

```python
# Let's make a child class that inherits from Animal!

class Dragon(Animal): # The bracket makes Dragon a subclass of Animal
    _breath = ""
    
    def __init__(self, name, height, weight, sound, breath):
        self._breath = breath
        # The super keyword lets the parent/superclass handle what you tell it to!
        super(Dragon, self).__init__(name, height, weight, sound)
        
        # You COULD also write it this way, but it means remembering the name of the parent class
        # Animal.__init__(self, name, height, weight, sound)
        
    def set_breath(self, breath):
        self._breath = breath
        
    def get_breath(self):
        return self._breath
    
    # These are different from those in Animal! They're 'polymorphisms' that overwrite the
    # implementation in the parent class!
    def get_type(self):
        print("Dragon")

    def __str__(self):
        return "{} is {} cm tall, {} kg, says {}, and breathes {}".format(self._name,
                                                                            self._height,
                                                                            self._weight,
                                                                            self._sound,
                                                                            self._breath)
    
    # Let's go through method overloading also
    
    # Which is a way for you to let methods behave differently depending on the inputs
    # There's no inbuilt way to do this in Python, but you can get around it with default arguments
    def multiple_sounds(self, times = None):
        if times is None:
            print(self.get_sound())
            return
            # Note that this works because get_sound is implemented in the parent class, Animal!
        else:
            print(self.get_sound(),str(times),"times!")
```

```python
# Let's make a dragon! (Properties stated not representative of the actual methylDragon)
methylDragon = Dragon("methylDragon", 201, 80, "Rawr", "The Breath Of Song")

print(methylDragon)
# Prints: methylDragon is 201 cm tall, 80 kg, says Rawr, and breathes The Breath Of Song

# Let's test our overloaded method!
methylDragon.multiple_sounds() # Prints: Rawr
methylDragon.multple_sounds(2) # Prints: Rawr 2 times!
```



### 2.4 Polymorphisms <a name="2.4"></a>

[go to top](#top)

> There's no such thing as an explicitly Virtual Method in Python (ala C++), because everything is duck typed. So I won't talk about virtual methods here (check my C++ OOP reference if you want to know more about it!.) If you REALLY wanted to use them though, check the `abc` (abstract class) module.

Did you notice something back in section 2.2? There were different implementations of the get_type method!

```python
# Animal

def get_type(self):
    print("Animal")

# Dragon

def get_type(self):
    print("Dragon")
```

Sure enough, if you call them

```python
cat = Animal("Cate", 33, 10, "Meow")
methylDragon = Dragon("methylDragon", 201, 80, "Rawr", "The Breath Of Song")

cat.get_type() # Prints: Animal
methylDragon.get_type() # Prints: Dragon

# This is the gist of what polymorphisms are!
# Methods with the same name, but with different implementations!
```



### 2.5 Magic Methods <a name="2.5"></a>

[go to top](#top)

You might have noticed that the methods like `__init__(self)` `__str__(self)`, etc. seem kinda special.

That's because they are! They're part of a group of pre-defined methods called **magic methods**.

There's one more I'd like to tell you about, but the rest of them are here: https://rszalski.github.io/magicmethods/

```python
# If __init__(self) is the constructor method

# __del__(self) is the destructor! It gets called each time an object is uninstantiated/destroyed

def __del__(self):
    print("Object being destroyed")
```



### 2.6 Generating Multiple Objects at a Time<a name="2.6"></a>

[go to top](#top)

Let's say you're lazy and you want to make multiple objects without having to type them out one by one.

**Use a list comprehension and make them in a list!**

```python
objs = [my_class() for i in range(10)]
for obj in objs:
    other_object.append(obj)

objs[0].do_sth()
```

```python
# Here's a more fleshed out example

from math import sqrt

class Coordinate:
	x = 0
	y = 0

def area_of_triangle(p1,p2,p3):
	side = lambda a, b : sqrt((b.x-a.x)**2  + (b.y-a.y)**2)
	side_one = side(p1,p2)
	side_two = side(p2,p3)
	side_three = side(p3,p1)
	s = (side_one + side_two + side_three)/2
	area = sqrt(s*(s-side_one)*(s-side_two)*(s-side_three))
	return round(area,2)

points = list() # Generate a list for Coordinate objects

for num in range(3):
    points.append(Coordinate()) # Create a new Coordinate object and append it to the list
    # Set Coordinate parameters
    points[num].x = float(input(f"Enter x coordinate of the #{num+1} point of a triangle: "))
    points[num].y = float(input(f"Enter y coordinate of the #{num+1} point of a triangle: "))

print("Area of triangle:",area_of_triangle(points[0],points[1],points[2]))
```

**Or a dictionary comprehension!**

```python
class my_class:
   def __init__(self, name):
       self.name = name

instance_names = ['Steven', 'Bob', 'Sophie']

# Dictionary!
objects = {name: my_class(name=name) for name in instance_names}

# Access as such
print(objects["Steven"].name) # Prints "Steven"
```







```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\     
```

