# My 2012-2013 Collegiate Robotics Club Experience

## Spring 2012

I joined the Robotics Club in Spring 2012 and got heavily involved in upper
level software framework and algorithm design.  At the time, there were no
designated responsibilities, so I ended up designing and writing the whole
upper level framework and most of the algorithms, with some help for
path-planning.  Using this code, the Robotics Club entered a regional
competition and performed the best they ever had.  Although the [2012
competition rules](Competition/MICS_2012.pdf) were different from the [2013
competition rules](Competition/MICS_2013.pdf), the software framework developed
in Spring 2012 would prove to be useful in the 2013 competition, since the
physical competition environment was the same.  I've included an old [Linux
executable file](Misc/GridBot_Spring2012) demonstrating the upper-level logic
from last year so you can see how it worked.  You can select from tests 1-7
when prompted.  Test 6 showcases a bug if you're interested in seeing one.

## Summer 2012

In Summer 2012, I took an A.I. class from [redacted] and became engrossed in
LISP, reading _The Little Schemer_ and parts of _The Joy of Clojure_ for
background information for a class project.  In this enthusiasm, I wrote a
[dynamic branch-and-bound path-planning algorithm in Scheme](Misc/return_home.scm)
for future use in Robotics Club.  The algorithm was designed to move the robot
from the "goal" floor tile to the "home" floor tile in the fastest way the
robot could possibly figure out based on what it knew about its environment
(i.e. what it had already sensed).  You may run this Scheme file if you wish --
there is a documentation block at the top of the source file on how to do so.
I also did a [write-up](Documentation/UL_Description.pdf) that described the
Spring 2012 competition rules, the upper-level software framework, and the
design of the "return-home" algorithm.  This document gives a good visual
introduction to the large code base for 2013, even though it did change, as
described later.

## Fall 2012

I took a break from Robotics Club in Fall 2012, given my credit load.  However,
in that time, I tried porting over the return-home algorithm from Scheme to C
(the native language of the AVR microcontroller used by the robot).  I quickly
found that translation of any non-trivial program from Scheme to C is very
difficult, and I wasn't familiar enough with advanced C features (e.g. void
pointers, function pointers, and preprocessor directives) to do this
translation easily, so I picked up and scanned through K&R.  I eventually
became fed-up (with the translation -- not K&R) and wanted to use a compiler to
automate this process, but could not find an existing tool that translated my
program to a small enough C file to fit on the microcontroller (mainly due to
library dependencies).  I put this project on the backburner until I became an
active member of Robotics Club again in the spring -- I'll explain how I
resumed this later on in the email.

## Spring 2013

In Spring 2013, I became the designated leader of the upper-level software
design group for Robotics Club, and with it came the responsibilities of
framework design, documentation, programming, testing, group organization, and
mentoring / training group members.  Active members of this team included
[redacted], who were all new to Robotics Club.  Throughout the last half of the
semester, I spent considerable time (1 hour / week on average) with them
explaining the code base, programming concepts, and team framework techniques.
I also facilitated group brainstorming and algorithm design sessions for
creating the algorithm to move the robot from the start location to goal
location.  I have included some of my own
[documentation](Documentation/move_to_flag.txt) from this
[brainstorming period](Documentation/ExampleBrainstorming.txt).

I must give special recognition to [redacted], who proved to be very proficient
at navigating the existing code base by the end of the semester, as he put in
the most time and effort of the new members and was very inquisitive.  We ended
up using his code in the competition for transporting the robot from its
initial position to the enemy flag.  However, note that in the `GridBot.c`
which I included in this repository, the robot follows my own buggy algorithm I
wrote for this.  I chose to include my version so that I can confidently say
that the contents of `GridBot.c` and all other included files except for those
in the `Competitions` subdirectory were written completely by me.  You will
find bugs for this part when you run the simulation (which I'll explain later).

Besides mentoring and leading discussions, I had at least three major
responsibilities in the Spring 2013 robotics club:

1. Porting the "return-home" algorithm from Scheme to C and integrating it into
   the existing framework
2. Reworking the framework by trimming fat and increasing flexibility
3. Establishing better development and testing tools

I will now describe each of these responsibilities in detail.

### 1. Return Home Algorithm

As stated before, I tried porting the return-home algorithm from Scheme to C in
Fall 2012, with no positive results.  I also tried to find a suitable compiler
but could not find one.  In Spring 2013, I resumed my search for a compiler,
spending an entire Saturday scouring the Internet for one, with no luck.  I
then decided it may be a good idea to try to write my own partial Scheme-to-C
compiler (I really didn't want to port the code by hand!).  I had started
writing one in Summer 2012 for fun, so I thought, why not?  After many
scattered sessions of trial, error, and burn-out over the semester, I finally
made some substantial progress at the beginning of April -- I got a program to
work that translated simple Scheme statements to C.  The following keywords
worked for heterogeneous lists:

- `car`
- `cdr`
- `cons`
- `list`
- `quote`

I was only missing `define`, `lambda`, `cond`, `else`, `equal`, `+`, and `-`
before I could have translated my Scheme source file to C (with a bit of
modifications before and after).  But then disaster struck -- I lost all my
source code I made up to that point after my laptop battery died during a
shutdown.  I didn't have it backed up to cloud storage, and the Vim backup
mechanism failed.

Disheartened, I didn't want to do it over again, so I got to thinking about
the alternative of porting the return-home algorithm by hand again.  In my
sober state-of-mind, I realized that the compiler I was writing would have had
terrible performance as compared to an ad-hoc solution.  My _beautiful_,
_elegant_ (in my mind) Scheme-to-C compiler was doing more than I needed it to:

1. It allowed for heterogeneous types, but it required void pointers and casts
   all over the place
2. It restricted most programming to construction of recursive lambdas building
   up from primitives like `car`, `cdr`, and `cons`, but introduced heavy
   overhead of frequently pushing and popping activation records without any
   tail-call optimization.

This whole experience taught me quite a bit about programming languages:
abstraction is always good in theory, but the overhead complexity to achieve
the abstraction can be quite drastic at a lower level.  I finally understood
why C is considered to be such a good programming language -- there is hardly
any overhead in the language constructs, even with a simple C compiler, since
the semantics follows the von Neumann architecture so closely.  Most of the
overhead is incurred by the programmer instead of the language... which I
suppose can be interpreted multiple ways.

I then got to work with porting over the "return-home" algorithm by hand.  I
finally did it, and you can see the code in `GridBot.c`.  The design is briefly
explained in
[Documentation/UL\_Description.pdf](Documentation/UL_Description.pdf).
Basically, it implements a dynamic branch-and-bound algorithm using a quad tree
(one child node for each cardinal direction), trying to find the shortest path
from point A to point B in terms of _combined number of forward steps and
90-degree turns_.  Any time the robot senses something different than it
remembers while following the calculated path, it will perform the
branch-and-bound search again.  If there is no way to get to point B from point
A, the robot will [semi-]intelligently wall-hug the boundary in its way until
it senses an opening.

## 2. Framework Overhaul

Because the framework that I wrote in 2012 was written to be flexible, all
development for 2013 took place with the same framework -- however, that
framework changed by subtraction and addition of thousands of lines of code in
both directions, and different sections were added to the framework.  For 2013,
the framework was split up into the following sections:

- Compile Flags (**new for 2013**)
- Platform-Dependent Macros (**new for 2013**)
- Declarations / Global Variables
- Main Routine
- Initialization and Deallocation Functions
- Sensor Functions
- Actuator Functions
- Robot Memory Check Functions
- Patterned Movement Functions
- Return-Home Functions (**new for 2013**)
- Virtual Object Presence Functions (sensing of virtual obstacles -- used to
  test the upper-level logic in isolation from hardware)
- Console Output Test Functions

Because \[redacted\] (the leader of lower-level software / hardware group) was
having trouble with including header files in the AVR project, unfortunately
the whole framework had to be written in one
[ANSI C source file](Spring2013_Code/GridBot.c).  [redacted] then had to
copy-paste his lower-level code into the source file at specified spots marked
by comments with agreed-upon syntax.  In order to achieve some level of
modularization and information-hiding, each section of the framework was
contained in its own textual "fold", defined using Vim syntax.

**SIDE NOTE**: I highly recommend browsing the source file using Vim.  This
will enable you to navigate with ease using textual folds.  If you don't
already know them, here are the keybindings:

- `zo`: open fold under cursor
- `zc`: close fold under cursor
- `zr`: open all folds in file
- `zm`: close all folds in file

As you can see above, I added a "Compile Flags" section to the framework.  By
disabling compile flags in the source code (i.e. commenting-out `#define`
directives), features could be excluded from the executable after compilation,
which both increased the flexibility of the source file and optimized the
executable for size.  For example, [redacted] was experimenting with increasing
the number of sensors on the robot, but wasn't sure how much he could get
working, so I created compile flags that could be set according to what sensors
were available.  The code that would be affected by availability of sensors
would then compile according to which flags were set, using `#ifdef` and
related directives.  Another use of compile flags would be to set the mode of
the executable -- that is, whether it was meant for use on the AVR controller
or in a console window of a PC.  Also, there were compile flags that allowed
the AVR executable to have different debugging features, like serial monitor
output and disabling of actuators.  The full documentation for the compile
flags can be seen in the source code.

Another part of the framework I added in Spring 2013 was a "Platform-Dependent
Macros" section, which is just like how it sounds.  By using appropriate
compile flags, the "Platform-Dependent Macros" section accounted for the
following environments:

- Linux
- Windows (partial support -- dropped)
- AVR
- Arduino (dropped)

I spent significant time (20 hours?) along with [redacted] trying to get the
source code to work on Arduino -- I even bought a book for this purpose -- but
we had no luck.  The "platform dependency" having to do with Arduino seemed to
be mostly language differences.  Arduino uses a quasi-C++ that required
extensive use of platform-dependent macros for us to get the Robotics Club
framework working on it, and even though I thought I understood it, the
roadblocks became too many, so we decided to just stick with AVR (looking back,
I wish we had done more with Arduino -- hardware problems caused our demise
this year at the competition).  Also, I wanted to add support for Windows so
that newer Robotics Club members unfamiliar with Linux could have contributed
more, but I didn't have the time for that unfortunately.  There was Windows
support last year but the overhaul of testing tools would have required more
time spent on developing portability.

## 3. Development and Testing Tools

First of all, I wanted to make testing the upper-level logic of the robot
easier this year than last.  If you run our executable from last year, you can
see that test cases can be selected by entering a number at the beginning.
There are many disadvantages to the approach, including that the individual
test cases have to be manually written into the source code and the then the
source has to be recompiled before testing, and also that the user cannot see
the arrangement of obstacles in a test case before selecting one.  This year, I
developed the code to make the virtual test grid interactive in placement of
obstacles.  Once the executable is started inside a terminal, the user may
place and remove obstacles wherever he or she chooses.  I started to write a
system for saving and loading files specifying arrangements of obstacles, but I
ran into a snag related to a side-effect from `termios` (terminal I/O settings)
changes that I made for another bit of code.  This is a feature that I'd like
to add for fun in the future.

Another change that I made to the virtual grid from last year is that you can
control the enemy robot on the grid (of course, last year, there was no enemy
robot on the grid to begin with).  The enemy robot does not follow the same
rules on the virtual grid that our robot does -- it can pass through obstacles
and jump over our robot -- but that is okay because the only purpose of
controlling the enemy robot is to test our robot's interaction with it.

I have written [documentation](Documentation/VirtualBotSymbols.txt) for
semantics of ASCII symbols in the virtual grid and
[documentation](Documentation/VirtualBotControls.txt) for the interactive
controls.  **Without referencing these documents, it will be hard to know what
is going on or how to interact with the simulation**.

In addition to overhauling the virtual grid simulation, I created a [build
script](Spring2013_Code/build.sh) to speed up development.  Quoting from
[Spring2013\_Code/README.txt](Spring2013_Code/README.txt):

> To build GridBot.c:
>
>     $ sh build.sh
>
> The build will output two executable files:
>
>     1. AVRBot:     executable to be run on AVR microcontroller
>     2. VirtualBot:  executable to be run in \*nix shell
>
> The build will also print the size of AVRBot and VirtualBot to standout output.

There are more files included in this repo than I have mentioned -- browse them
as you please.
