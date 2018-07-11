# WordPress Notes

Author: methylDragon  
Contains notes on WordPress in general, plugin development, and a code reference for actions and filters

***

## Pre-Requisites

### Good to know

- **PHP** (read my PHP syntax reference!)



## Table Of Contents <a name="top"></a>

1. [Introduction](#1)  
2. [Plugin Development](#2)    
   2.1 [Introduction](#2.1)    
   2.2 [Boilerplate Code](#2.2)    
   2.3   [Actions, Filters, and Hooks](#2.3)    
   2.4   [A Refresher on PHP Functions](#2.4)    
   2.5   [Hooks](#2.5)    
   2.6   [Actions](#2.6)    
   2.7   [do_action() and do_action_ref_array()](#2.7)    
   2.8   [add_action()](#2.8)    
   2.9   [Unhooking actions](#2.9)    
   2.10 [Miscellaneous action related functions](#2.10)    
   2.11 [Filters](#2.11)    
   2.12 [apply_filters()](#2.12)    
   2.13 [add_filter()](#2.13)    
   2.14 [Unhooking filters](#2.14)    
   2.15 [Miscellaneous filter related functions](#2.15)    
3. [Relevant Links](#3)    



## 1. Introduction <a name="1"></a> 

WordPress (WP) is a content management system that uses PHP to dynamically generate pages from its MySQL database.

Since it was started as a blogging platform, WordPress sites are organized into **pages** (eg. activity, about page, etc.) and **posts** (blog posts.) WP provides an easy backend console to deal with this and create pages and posts, but its true value comes in its extendibility with plugins and customization with code.



# 2. Plugin Development <a name="2"></a>

### 2.1 Introduction <a name="2.1"></a>

[go to top](#top)

The main bulk of plugin development will deal with introducing new **actions** and **functions** hooked onto the WP core, as well as coding in custom PHP and MySQL queries to interact with, and modifying the MySQL database that WP comes packed in with. **Plugins are written in PHP.** Wrap them in `<?php` and `?>` tags!



### 2.2 Boilerplate Code <a name="2.2"></a>

[go to top](#top)

In order for WP to recognize that a .PHP file is a plugin, you need to ensure that you include a comment detailing the description and other metadata for the plugin as a whole!

```php
/*
Plugin Name: <PLUGIN NAME HERE>
Plugin URI: <PLUGIN DESCRIPTION URL>
Description: <PLUGIN DESCRIPTION>
Version: <PLUGIN VERSION>
Text Domain: <FOR TRANSLATIONS>
Domain Path: <FILE DIRECTORY FOR TRANSLATIONS>
Author: methylDragon
Author URI: methylDragon.com
License: <LICENSE>
*/
```

Also, **FYI**, whenever you open a parenthesis, make sure you add a space after it, otherwise WP will reject the code...

`some_function( 'see_the_parentheses?' );`



### 2.3 Actions, Filters, and Hooks <a name="2.3"></a>

[go to top](#top)

In order to deal with constant WP updates while maintaining plugin compatibility, WP has implemented a system called **<u>Actions, Filters, and Hooks</u>** to do this. This will form the bulk of any plugin development.



### 2.4 A Refresher on PHP Functions <a name="2.4"></a>

[go to top](#top)

Here's a PHP function prototype refresher!

```php
function functionName() {
    code to be executed;
} 
```

You can call functions directly on WP using functionName(); , or call them via an action, or filter! You'll see why this is useful later on.



### 2.5 Hooks <a name="2.5"></a>

[go to top](#top)

> Hooks are provided by WordPress to allow your plugin to 'hook into' the rest of WordPress; that is, to call functions in your plugin at specific times, and thereby set your plugin in motion. (From WP codex) 

These hooks exist in two forms, Actions, and Filters.



### 2.6 Actions: Call PHP functions that are supposed to <u>do something <a name="2.6"></a></u>

[go to top](#top)

Actions are places where WP or other plugins call certain functions. You *can* write your own actions, and call them either alone, or *as a callback* alongside another action! The latter is the reason why an action can be a hook, since an action that you write can be hooked onto another action!

* The difference between calling a function directly and using an action, is that when you call a function using an action, any other actions that are hooked onto that action you just called will also end up being called.




### 2.7 do_action() and do_action_ref_array() <a name="2.7"></a>

[go to top](#top)

```php
/* Use do_action() to call a function anywhere you want

do_action( string $tag,  $arg = '' )

OPTIONAL: $arg (mixed) You can pass arguments to the action!
*/

	do_action( 'function_name' );

/* Use do_action_ref_array() to call a function anywhere you want, while passing an array as an argument!
*/

	$args = array( 'arg_1', true, 'foo', 'arg_4' );
	do_action_ref_array( 'function_name', $args );

	//Is equivalent to

	do_action( 'function_name', 'arg_1', true, 'foo', 'arg_4' );
```


### 2.8 add_action() <a name="2.8"></a>

[go to top](#top)

```php
/* Use add_action() to hook an action onto another existing action
It will call your hooked action whenever the first action is called

add_action( string $tag, callable $function_to_add, int $priority = 10, int $accepted_args = 1 )

OPTIONAL: $priority (int) Used to specify the order in which functions associated with a particular action are executed. Lower number means higher priority. Functions with the same priority number are executed in the order in which they were added to the action. Default = 10

OPTIONAL: $accepted_args (int) The number of arguments the function accepts. . Default = 1

NOTE: The arguments passed to the $tag action will be the ones passed to the $function_to_add
*/
	add_action( 'some_hook', 'callback_function_for_some_hook' );

// Or if calling a function from within a class
	add_action( 'some_hook', array( 'some_class', 'callback_function_in_some_class' ) );
```



### 2.9 Unhooking actions <a name="2.9"></a>

[go to top](#top)

```php
/* Use remove_action() to unhook an action
Note: You cannot call this directly, it has to be done from within a function!

This returns true when finished
*/
	function remove_my_action() {
		remove_action( 'some_hook', 'function_being_removed' );
	}
	do_action( 'remove_my_action' );

// Or if the action was hooked from within a class, eg. by a plugin
	function remove_my_class_action(){
		global $my_class;
		remove_action( 'some_hook', array( $my_class, 'class_function_being_removed' ) );
	do_action( 'remove_my_class_action');
	}

/* Use remove_all_actions() to unhook everything hooked to an action

This returns true when finished

Note: Becareful when using this, because it might cause issues with nested hooks!
*/
	remove_all_actions( 'some_hook' );
```



### 2.10 Miscellaneous action related functions <a name="2.10"></a>

[go to top](#top)

```php
/* Use has_action() to check if an action has been registered for a hook.
It returns a boolean value, or an integer, if the action has a priority value associated with it
*/
	has_action( 'hook_name', 'function_to_check' );

// Use did_action() to see how many times an action has fired
	did_action( 'action_name' )
```



### 2.11 Filters: Call PHP functions that <u>receive and returns values <a name="2.11"></a></u>

[go to top](#top)

Note: They might potentially modify the received value, but they **must** return a value

Otherwise, filters act a lot like actions, you can add, and do them just like actions, but with a few small differences. Just **make sure your functions defined to be called by filters take in and RETURN values!**

I'll add less detail for the filter functions because a lot of them are aliases for the action functions



### 2.12 apply_filters() <a name="2.12"></a>

[go to top](#top)

```php
/* An analog is do_action()

apply_filters( string $tag, mixed $value )

You must supply a parameter!
*/
	apply_filters( 'filter_to_use', 'value_to_filter', 'potentially_more_values' );
```



### 2.13 add_filter() <a name="2.13"></a>

[go to top](#top)

```php
/* So let's say you might want to add additional filters onto filter_to_use. You use add_filter to hook an additional filter function to an existing filter, much like add_action!

add_filter( string $tag, callable $function_to_add, int $priority = 10, int $accepted_args = 1 )

Same thing with priority and accepted args for add_action

Note: Don't forget to define your callback function!
*/
	function additional_filter( $example ) {
    	// Maybe modify $example in some way.
    	return $example;
	}

	add_filter( 'filter_to_use', 'additional_filter' );
```



### 2.14 Unhooking filters <a name="2.14"></a>

[go to top](#top)

```php
/* remove_filter(): Same thing with actions, self explanatory
Returns true when done
*/
	function remove_my_filter() {
		remove_filter( 'some_hook', 'function_being_removed' );
	}
	do_action( 'remove_my_filter' );

// Or if the filter was hooked from within a class, eg. by a plugin
	function remove_my_class_filter(){
		global $my_class;
		remove_filter( 'some_hook', array( $my_class, 'class_function_being_removed' ) );
	do_action( 'remove_my_class_filter' );
	}

// remove_all_filters() does exactly what you think it does
	remove_all_filters( 'some_hook' );
```



### 2.15 Miscellaneous filter related functions <a name="2.15"></a>

[go to top](#top)

```php
// current_filter() retrieves the name of the current filter or action

	//Retrieving filter name
		function my_filter() {
	  	  echo current_filter(); // 'the_content'
		}
		add_filter( 'the_content', 'my_filter' );

	//Retrieving action name
		function my_init_function() {
	  	  echo current_filter(); // 'init'
		}
		add_action( 'init', 'my_init_function' );

// has_filter() works like has_action(), returns true if a hook has the passed function hooked onto it
	has_filter( 'hook_name', 'function_to_check' )
```



## 3. Relevant links: <a name="3"></a> 

https://codex.wordpress.org/Plugin_API  
https://www.youtube.com/watch?v=AQrXcsc4CIw (Actions, Filters, and Hooks)  
https://www.youtube.com/watch?v=w257tYZtL8Y (WordPress Plugin Development)  
https://wordpress.stackexchange.com/questions/1007/difference-between-filter-and-action-hooks (Filters vs Actions)

​    

------

[![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)