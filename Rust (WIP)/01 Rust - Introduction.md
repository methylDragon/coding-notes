# Rust Crash Course

Author: methylDragon  
Contains a syntax reference for Rust!  
I'll be adapting it from several sources that will be credited in the introduction

------

## Pre-Requisites

**Assumed knowledge (This is a Rust crash course, not a basic coding tutorial)**

- How **variables, loops, conditionals, etc**. work (Basic coding fundamentals will help a lot!)
- Linux (**Terminal/Console proficiency**) (We're going to need to compile our stuff)
- Gone through the all preceding parts of the tutorial

### Good to know

- Other systems programming language knowledge (e.g. C++)
  - It'll help with appreciating the benefits of Rust!

- If you have knowledge of computation structures like variables, functions, OOP, etc. it'll be easier



## Introduction

This section of the tutorial walks you through the basics of Rust, including:

- An overview of what Rust is, why to use it, and how to set up your environment for developing with it
- The toolings Rust affords
- Common programming constructs in Rust



### Preamble: Here be Dragons

Rust has been described to be cognitively frontloaded, in that it has a very steep learning curve, but once you get it, it saves you a boatload of time. This should **not** be a deterrent to learning the language! And the reason why is...

Though it takes a little bit of time to get a Rust program to compile without compiler errors because the compiler is built specifically with safety in mind (enforced at compile time) to stop you from making mistakes, once you get a Rust program to successfully compile (and the compiler provides a lot of helpful warnings and notes to get you there), **it usually does what you want it to**.

>  In fact, it's virtually impossible to [encounter data races](https://doc.rust-lang.org/nomicon/races.html) or other [memory issues](https://stackoverflow.com/questions/36136201/how-does-rust-guarantee-memory-safety-and-prevent-segfaults) (e.g. segfaults, double-frees, null or dangling pointers))!

This is a little bit of a different paradigm from C++, where in C++ getting stuff to compile is (relatively) easy, but getting your program to be memory safe and to do what you want it to do will take a lot more time, and **without** a lot of guidance from the compiler!

I think the benefits **far exceed** the learning curve required to train yourself to stop fighting the Rust borrow-checker (explained later) and compiler. Though it will help tremendously to always keep in mind that it'll take a little bit of time to get used to the way Rust works, and to work with the compiler. Keep at it, and you'll be on your way to much safer, error free code in no time! And most importantly, I'll be here every step of the way ;)

Ready? Let's go!!



### What is Rust

> **Rust** is a [multi-paradigm](https://en.wikipedia.org/wiki/Multi-paradigm_programming_language), [general-purpose programming language](https://en.wikipedia.org/wiki/General-purpose_programming_language) designed for [performance](https://en.wikipedia.org/wiki/Computer_performance) and safety, especially safe [concurrency](https://en.wikipedia.org/wiki/Concurrency_(computer_science)). Rust is [syntactically](https://en.wikipedia.org/wiki/Syntax_(programming_languages)) similar to [C++](https://en.wikipedia.org/wiki/C%2B%2B), but can guarantee [memory safety](https://en.wikipedia.org/wiki/Memory_safety) by using a *borrow checker* to validate [references](https://en.wikipedia.org/wiki/Reference_(computer_science)).
>
> Rust achieves memory safety without [garbage collection](https://en.wikipedia.org/wiki/Garbage_collection_(computer_science)), and [reference counting](https://en.wikipedia.org/wiki/Reference_counting) is optional. Rust has been called a [systems programming](https://en.wikipedia.org/wiki/Systems_programming) language, and in addition to high-level features such as [functional programming](https://en.wikipedia.org/wiki/Functional_programming) it also offers mechanisms for [low-level](https://en.wikipedia.org/wiki/Low-level_programming_language) [memory management](https://en.wikipedia.org/wiki/Memory_management).
>
> Source: [Wikipedia](https://en.wikipedia.org/wiki/Rust_(programming_language))

What this means is, that with Rust, you get the same power and speed you can get with C/C++, but with memory safety as a default (we'll go into how later on!) With the tradeoff of a more strict compiler, you get the benefits of safety by default, and a much higher chance your programs will work as expected once they compile successfully!

Beyond that, being a modern, Rust provides some **significant advantages** over C++, built into the language, such as:

- A package and dependency manager that is **version aware** (like Python's `pip`)
  - No more manually managing dependencies and linking libraries!
- Modern tooling
  - Inbuilt testing (and resolution to unit and integration tests)
  - Inbuilt documentation generation (which also runs tests defined in the docstrings!!)

While Rust is still new, the ecosystem has stabilized a lot in the past few years, with large companies both adopting it and contributing to its libraries. It might not be as ubiquitous as C/C++, but it's very much a worthwhile language to learn!

>  Also, say hello to [Ferris the crab](https://rustacean.net/), the unofficial mascot of Rust! :crab:
>
> ![img](assets/01%20Rust%20-%20Introduction/ferris.gif)
>
> [Image Source](https://www.rust-lang.org/learn/get-started)



### Setup

#### **Rust Install**

[Rust Install](https://www.rust-lang.org/tools/install)

Ready to go? Let's set up your environment for Rust! (I'm on Ubuntu 20.04.)

We can install Rust using `rustup`, which manages updates. This script will settle everything for you, including setting up your PATH environments.

```shell
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

You can then go ahead and see what version of Rust you're running with

```shell
rustc --version
```



#### **Dev Tools**

Like with C++, development in Rust is made a lot easier with editors/IDEs. There's a [handy site](https://areweideyet.com/) that tracks support and integrations with some of the more commonly used ones out there.

For myself, I will be using [Atom](https://areweideyet.com/#atom), and this is the setup process for that.

1. Ensure that you've installed the following packages in Atom
   - `ide-rust`, `atom-build`, `build-cargo`
2. Ensure that you've installed the standalone dependency `rust-analyzer` for language support
   - I did this on Ubuntu 20.04 using the [snap store](https://snapcraft.io/install/rust-analyzer/ubuntu)
     - Hence, I needed to add `/snap/bin` to `PATH`!
   - But you might need a [manual install](https://rust-analyzer.github.io/manual.html#installation) on other platforms



### Credits

These sources were heavily used in the construction of this reference. Some of the content has been reflected verbatim (which should be credited or placed in block-quotes) or paraphrased.

These sources are also recommended reading, in the following order:

- The ever amazing Derek Banas' [Rust Tutorial Video](https://www.youtube.com/watch?v=U1EFgCNLDB8)
- [Gentle Intro to Rust](https://stevedonovan.github.io/rust-gentle-intro/readme.html)
- [Rust by Example](https://doc.rust-lang.org/rust-by-example/index.html)
- [The Book](https://doc.rust-lang.org/book/title-page.html)

Other minor sources will be linked as they are used.



### More Recommended Reading

- If you're still considering Rust, watch [this video](https://www.youtube.com/watch?v=DnT-LUQgc7s), it does a brief rundown of the features and pros and cons.
- If you're interested in recipes: [Cookin' with Rust](https://rust-lang-nursery.github.io/rust-cookbook/intro.html)
- If you need performance: [The Rust Performance Book](https://nnethercote.github.io/perf-book/introduction.html) and section in [The rustc Book](https://doc.rust-lang.org/rustc/profile-guided-optimization.html)
- If you're interested in unsafe Rust: [The Rustonomicon](https://doc.rust-lang.org/nomicon/intro.html)
- If you want to cross-compile: [The Rustup Book](https://rust-lang.github.io/rustup/cross-compilation.html)
- If you want to use Rust code in C/C++: [The `bindgen` User Guide](https://rust-lang.github.io/rust-bindgen/introduction.html)



## Rust Tooling (Basic)

This is just a very cursory look at the toolings available to you in Rust so you can get started. It can go a whole lot deeper, but now is not the time to go through that (perhaps in a different tutorial or section.)

Just be a teeny bit patient, we'll get to the syntax reference soon! It'll be very good to familiarize yourself with the toolings Rust affords you, and how to do several common things!



### Running a Program

#### **Using `rustc`**

The most basic way to run a Rust program is:

```shell
$ rustc <filename>.rs
$ ./<filename>
```



#### **Using `cargo` (Better)**

You can also use the inbuilt package manager, `cargo` to generate a project and run it!

```shell
# Create a new Rust project (with a hello world source!)
$ cargo new hello-rust

# Compile it (run will build it if not built, and then run it)
$ cd hello-rust
$ cargo run
```

You can also build and test it with:

- `cargo build`
- `cargo test`

(I much prefer using my IDE to invoke `cargo` though.)



### Managing Dependencies (Basics)

#### **Crates**

Rust comes with its own package manager (and build system, ... and test runners), `cargo`! It makes it ridiculously easy to download, build, link, and publish packages (called **crates**).

> **Crates**
>
> Crates are units of compiled code, which can either be executables or libraries!



#### **Specifying and Using Dependencies**

You specify the dependencies of a Rust project in its `Cargo.toml` file.

Sample `Cargo.toml` (`#` denotes comments!):

```toml
[package]
name = "some_package"
version = "0.1.0"
edition = "2021"

[dependencies]
# Local Dep
some_local_package = { path = "../some_local_package" }

# GitHub Dep
zenoh = { git = "https://github.com/eclipse-zenoh/zenoh.git" }

# From crates.io
env_logger = "0.9.0"
async-std = { version = ">=1.9.0, <2.0.0", default-features = false, features = [
    "attributes",
    "unstable",
] }
```

We see four different ways to be specifying a dependency! Respectively:

- Local
- GitHub
- A crate of a specific version from the ecosystem (crate.io)
- A crate of a range of versions from the ecosystem (crate.io), with additional parameters specifying which features (think sub-modules/components) of the package you're using

There's a lot more to dependency management, but this should be enough to start you off using crates from the ecosystem!



#### **Cargo Lock Files**

> **`Cargo.toml` vs `Cargo.lock`**
>
> - `Cargo.toml` is about describing your dependencies in a broad sense, and is written by you.
> - `Cargo.lock` contains exact information about your dependencies. It is maintained by Cargo and **should not be manually edited**.
>
> [Source](https://doc.rust-lang.org/cargo/guide/cargo-toml-vs-cargo-lock.html)

The lock files allow you to have reproducible builds! They disambiguate any ambiguous versions in the `Cargo.toml` file to specific versions used for the build you just had. This lets you share the `Cargo.lock` file to let others reproduce your build (using the `--locked` flag to use it!)



### Package Layout

> ```
> .
> ├── Cargo.lock
> ├── Cargo.toml
> ├── src/
> │   ├── lib.rs
> │   ├── main.rs
> │   └── bin/
> │       ├── named-executable.rs
> │       ├── another-executable.rs
> │       └── multi-file-executable/
> │           ├── main.rs
> │           └── some_module.rs
> ├── benches/
> │   ├── large-input.rs
> │   └── multi-file-bench/
> │       ├── main.rs
> │       └── bench_module.rs
> ├── examples/
> │   ├── simple.rs
> │   └── multi-file-example/
> │       ├── main.rs
> │       └── ex_module.rs
> └── tests/
>     ├── some-integration-tests.rs
>     └── multi-file-test/
>         ├── main.rs
>         └── test_module.rs
> ```
>
> - `Cargo.toml` and `Cargo.lock` are stored in the root of your package (*package root*).
> - Source code goes in the `src` directory.
>   - The default library file is `src/lib.rs`.
>   - The default executable file is `src/main.rs`
>     - Other executables can be placed in `src/bin/`.
> - Benchmarks go in the `benches` directory.
> - Examples go in the `examples` directory.
> - Integration tests go in the `tests` directory.
>
> [Section from The Book](https://doc.rust-lang.org/cargo/guide/project-layout.html)



### Naming and Casing

Let's use the [Rust API guidelines](https://rust-lang.github.io/api-guidelines/naming.html)!

Briefly:

- `snake_case` for variables, functions, and modules
- `UpperCamelCase` for types, traits, and enum variants (think classes and OOP)
- `SCREAMING_SNAKE_CASE`

For crate and crate folder names, it's a bit more ambiguous, there's no standard, and crates.io has a mix of these:

- Both `kebab-case` and `snake_case` works, but I'd prefer `snake_case`, in case it matters (:
- You might even be able to mix them if you have a specific reason to do so (e.g. `project-crate_name`)



### More Toolings Resources

If you're interested in looking at more Rust toolings, you can take a look at these resources:

- [Rust Gentle Intro](https://stevedonovan.github.io/rust-gentle-intro) has:
  - A [section](https://stevedonovan.github.io/rust-gentle-intro/4-modules.html) on dynamically linking crates and stripping them down
  - More cargo magic
- [The Cargo Book](https://doc.rust-lang.org/cargo/index.html)
- [The Rust Embedded Book](https://docs.rust-embedded.org/book) has:
  - A [section](https://docs.rust-embedded.org/book/interoperability/rust-with-c.html) on using Rust with C/C++ (to call from C/C++)



## Rust Basic Syntax Reference

### Comments

Comments are meant to increase readability for programmers, and will be ignored by computers!

```rust
// This is a comment

/*
multiline
comment!
*/

/*
 * alternate multiline
 * comment! (Purely stylistic choice)
 */
```

> **Doc Comments**
>
> There are special [doc comments](https://doc.rust-lang.org/reference/comments.html#doc-comments) that mean special things for the documentation generator (for generating HTML documentation!) We won't go through in-depth what they do here, but it's best to know them so you can avoid using them and unwittingly do something you don't intend.
>
> Generally:
>
> - `///`
> - `//!`
> - `/**`
> - `/*!`
>
> ```rust
> /// Doc for function following this block
> /// # With Markdown Formattings
> ///
> /// ```
> /// // And code-blocks
> /// // Will ACTUALLY run this when testing
> /// assert!(true, "");
> /// ```
> 
> /** The block version
> */
> ```
>
> ```rust
> //! Standalone doc for this item
> //!
> //! Cool.
> 
> /*! The multiline version
> */
> ```



### Importing Libraries

After you've declared a crate as a dependency, you can import it!

```rust
// Import the whole module
use std::println;

// Import one member
use std::io::stdout;

// Import multiple members
use std::io::{stdout, BufWriter};

// Import with alias
use deeply::nested::function as other_function;
```

Cool! Now we use the imported names, like...

```rust
println!(...);
stdout();
BufWriter::new(...);
other_function();
```



### Macros (Basic)

Sometimes you'll see what looks like function calls, but suffixed with an `!`. E.g. `println!()`

These aren't true function calls, rather, they are **macros** (or at least, one type of Rust macro... We won't go through the rest here.)

These are declarative function-like macros, which **cause code to be generated/substituted in at compile time** that is then built and run, possibly with passed in arguments. What this means is that it helps you remove a lot of boilerplate!



### Hello World

```shell
$ cargo init hello-world
$ cd hello-world
$ cargo run
```

The `cargo init` command generates the hello world code in `hello-world/src/main.rs`

**Every Rust program consists of a <u>main function</u>**

```rust
fn main() {
	println!("Hello, World!");
}
```



### Printing

Some examples adapted from [Rust By Example](https://doc.rust-lang.org/rust-by-example/hello/print.html)

You can print to console (`stdout`) or error (`stderr`) with, respectively:

- `print!()`, `println!()`
- `eprint!()`, `eprintln!()`

The variants suffixed with `ln` appends a newline to the print.

```rust
println!("Rar");
```

Naturally, you can format the output as well (it's the reason why we're using macros!)

Here are some common uses, though it isn't [all of them](https://doc.rust-lang.org/std/fmt/index.html)!

```rust
println!("I say {}", "Rar");
println!("Stringify num: {}", 42);
println!("Stringify num, IN BINARY: {:b}", 2);  // Other formats work
println!("Stringify num, to some precision {:.5}", 0.1);  // 0.10000

println!("{0}{1}", "methyl", "Dragon");  // Positional arguments!
println!("Named {arg}", arg="arguments");

// Captured vars
let outside_var = "OUTSIDE";
println!("Print from {outside_var}!");  // "Print from OUTSIDE!"

// Padding
println!("{number:>width$}", number=1, width=6);  // "     1"
println!("{number:>width$}", number=1, width=6);  // "000001"
```



### Variables and Data Types

**Variables are like containers for data**

You declare them with the `let` keyword.

In Rust, variables are strongly typed. That is, every variable must be of some type. The cool thing, though, is for primitive types, Rust will **infer it for you** and automatically assign the type!

``` rust
let num = 10;                  // i32
let float = 10.0;              // f64
let string = "totally legal";  // &str
let bool = true;               // bool

// You can use _ as a separator!
let num = 11_22_33_44          // Becomes 11223344

// This throws error, since the default is i32 and it overflows
let num = 2147483647 + 1;
```

> **Note**: You can [use language keywords as identifiers](https://doc.rust-lang.org/rust-by-example/compatibility/raw_identifiers.html). This is to allow using older libraries with newer versions of Rust.
>
> But... Please don't, it'll make your code uneccesarilly confusing.



#### **Data Types**

Here's a selection of some of the more commonly used primitive types you'll encounter.

| Description                 | Types                                                        |
| --------------------------- | ------------------------------------------------------------ |
| Signed Integers             | `i8`, `i16`, `i32`, `i64`, `i128`, `isize` (platform dependent) |
| Unsigned Integers           | `u8`, `u16`, `u32`, `u64`, `u128`, `usize` (platform dependent) |
| Floating Point              | `f32`, `f64`                                                 |
| Characters (4-bytes)        | `char`                                                       |
| Boolean (`true` or `false`) | `bool`                                                       |

**Explicit Typing**

You can explicitly specify the type a variable should take! There's several ways to do this:

```rust
let float: f64 = 1.0;     // Regular
let num = 100i32;         // Suffix

let another_num = 100_i32 // With optional underscore separator
```

**Type Inference**

The Rust compiler is also smart enough to infer types from context!

```rust
// Source: https://doc.rust-lang.org/rust-by-example/primitives.html
let mut inferred_type = 12;  // i64 inferred from context
inferred_type = 4294967296i64;
```

**Literals**

Adapted from [The Book](https://doc.rust-lang.org/book/ch03-02-data-types.html).

```rust
let hex = 0xff;         // 255
let octal = 0o377;      // 255
let bin = 0b1111_1111;  // 255
let byte = b'A';        // u8 only: 65 

let c = 'c';  // char is differentiated from string with single quote
let s = "s";  // Double quotes
```



#### **Mutable, Immutable**

**By default, Rust variable values are immutable**! You can't change them once you set them unless you explicitly declare them to be mutable.

```rust
let immutable_num = 10;
let mut mutable_num = 20;

// This throws error
immutable_num = 20;

// This doesn't
mutable_num = 10;
```

And for obvious reasons, the type of a variable, even if it is mutable, **cannot be changed**

```rust
// This throws error
mutable_num = true;
```



#### **Shadowing**

However, **BEWARE**, you may shadow variables!

```rust
let num = 10;
let num = 20;  // This works! You're defining a new variable in a new memory location, afterall
```

I really don't think you should be doing this though.



#### **Const**

So then, if variables are immutable by default, what the heck is the `const` keyword supposed to be?

Well, `const` and immutable variables are different!

- `const` declarations are compile-time constants that are immediately **inlined** (substituted) before parsing for compilation with their values without any name binding during runtime (think macro substitutions)
  - Good candidates for creating in the global scope
- Immutable variables are created and evaluated at runtime, so there is a name binding
  - You can't create them in the global scope with `let`!

For more in-depth info, see this [StackOverflow post](https://stackoverflow.com/a/37881340), or the [docs](https://doc.rust-lang.org/std/keyword.const.html).



### Variable Introspection

A selection of handy things you can do.

**Get Numeric Type Bounds**

The same applies to all the other numeric types, including unsigned, floats, and pointer-size (`size`).

```rust
i8::MAX;
i8::MIN;
```

**Get Type Names and Addresses**

> This is meant for diagnostic use only, you shouldn't be relying on this because sometimes it won't function as expected!

```rust
fn type_of<T>(_: &T) {
    println!("{}", std::any::type_name::<T>())
}

fn main() {
    // Get types
    type_of(&"wow");  // &str
    type_of(&42);     // i32
    type_of(&3.14);   // f64
    type_of(&true);   // bool
    
    // Get pointer addresses (yes, even literals have a location at runtime...)
    let n = 10;
    
    println!("{:p}", &"wow");
    println!("{:p}", &1);
    println!("{:p}", &n);
}
```

Nifty!



### Casting

There's no implicit casting in Rust, only explicit casting. With...

**`as`** (Not recommended)

> Beware overflows with `as`! Shoving a larger data type into a smaller one will cause overflows, with [varying effects](https://doc.rust-lang.org/rust-by-example/types/cast.html).

```rust
let num = 65 as u8;   // 65
let c = num as char;  // 'A'
```

Note, you can't always do a cast.

```rust
// This fails because char casts only work with u8, and 65 is i32
let c = 65 as char;
```



**`from`** (Recommended)

`from` forbids overflows! It's safer that way!

```rust
let num = 65;
let c = char::from(num);
```



### Arithmetic and Math

#### **Numeric**

**Basic Arithmetic**

```rust
+  // Add: 3 + 2 equals 5
-  // Subtract: 3 - 2 equals 1
*  // Multiply: 3 * 2 equals 6
/  // Divide: 3 / 2 equals 1.5
%  // Modulo: 3 % 2 equals 1 (Modulo returns the remainder!)

// These also work!
let mut n = 10;  // Mutable!!

n += 1;  // n = n + 1
n -= 1;  // n = n - 1
n *= 3;  // and so on...
n /= 3;
n %= 3;
```

> Notably, things like `n++` or `++n` do not work! They're deliberately left out to protect the programmer.

**Math Functions**

There's also a bunch of math functions too! Here are some relevant ones, see full listings in the respective primitive's docs (e.g. [f64](https://doc.rust-lang.org/std/primitive.f64.html).)

```rust
// Abs only works on variables, it fails to convert silently on literals!
let mut neg_num = -4;

// Basics
neg_num.abs()        // abs(-4) = 4
2_i32.pow(8)         // 2 ^ 8
4_i32.powf(0.5)      // 4 ^ 0.5 = 2 (Square root!)

2_f64.sqrt()         // Square root
2_f64.cbrt()         // Cube root

4_f64.max(5_f64)     // 5
4_f64.min(5_f64)     // 4

4_f64.is_nan()       // Check if NaN
4_f64.is_infinite()  // Check if inf
4_f64.is_finite()    // Check if finite

// Rounding
1.23_f64.round()     // Round to closest int: 1_f64
4.56_f64.round()     // 5_f64
4.56_f64.floor()     // Round down: 4_f64
1.23_f64.ceil()      // Round up: 2_f64
1.23_f64.trunc()     // Return only integer part of number
1.23_f64.fract()     // Return only fractional part of number

// Exponentials
2_f64.exp()          // e ^ 2
4_f64.exp2()         // 2 ^ 4
2_f64.ln()           // ln(2)
2_f64.log10()        // log10(2)
2_f64.log2()         // log2(2)

// Trigo (functions are in radians!)
90_f64.to_radians()
3.14_f64.to_degrees()

90_f64.to_radians().sin()
90_f64.to_radians().cos()
90_f64.to_radians().tan()

90_f64.to_radians().asin()
90_f64.to_radians().acos()
90_f64.to_radians().atan()
90_f64.to_radians().atan2()
```



#### **Useful Numeric Properties**

These properties are part of the primitive type! Examples of some salient ones:

```rust
f64::DIGITS        // Approx significant figures
f64::EPSILON       // Smallest difference for type between 1.0 and next num
f64::NAN
f64::INFINITY
f64::NEG_INFINITY
```



#### **String**

```rust
let mut a = "A".to_owned() + "B";  // "AB", of type: alloc::string::STring

// Now we can do this
a += "C";
```

The rationale for the `to_owned()` requires a concept that will come in later...



### Conditionals

**Comparison Operators**

```rust
== // equal to 
!= // NOT equal to
> // more than
< // less than
>= //  more than or equal to
<= // less than or equal to
```

**Logical Operators**

```rust
&& // AND
|| // OR
! // NOT
```

**Example Conditional**

>  Rust can handle parentheses around conditionals, but it'll throw warnings because it doesn't like it.

```rust
let dragon = "methyl";

if dragon == "methyl" {  // Automatically invokes dragon.eq("methyl")
    println!("You're looking at my reference!");  // Invokes this!
} else {
    println!("Oh no!");
}

// You can also chain them with else if
let n = 5;

if n < 5 {
	println!("a");
} else if n > 6 {
    println!("b");
} else {
    println!("c");  // Invokes this!
}
```



### Ternary Operators (WIP)

You can also spoof a ternary operator in Rust.






```
                            .     .
                         .  |\-^-/|  .    
                        /| } O.=.O { |\
```

​    

------

 [![Yeah! Buy the DRAGON a COFFEE!](../_assets/COFFEE%20BUTTON%20%E3%83%BE(%C2%B0%E2%88%87%C2%B0%5E).png)](https://www.buymeacoffee.com/methylDragon)

