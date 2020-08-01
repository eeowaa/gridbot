/*{{{ >>> README <<<
 * =================
 * - When editing this file, conform to the following basic formatting conventions:
 *      - All tabs are SPACES!!! (no '\t' characters)
 *              - In Vi[m], type ':se smarttab smartindent expandtab' or add settings to $VIMRC file
 *              - Visual Studio, Notepad++, Textmate, Emacs, etc. all have this capability, as well
 *      - 1 Tab = 4 Spaces
 *              - In Vi[m], type ':se ts=4 sw=4' or add settings to $VIMRC file
 *              - Visual Studio, Notepad++, Textmate, Emacs, etc. all have this capability, as well
 *      - Only use multiline comments (do NOT use '//') in order to conform to ANSI-C standard
 *      - Constants and enum values should be UPPERCASE
 *      - User-defined types should be UpperCamelCase
 *      - Functions and variable names should be lowerCamelCase
 *      - Functions with no parameters should have 'void' as the argument
 * - Textual folds have been embedded for Vi[m] for ease of code navigation
 *      - Type ':se foldmethod=marker' to enable folds
 *      - The following commands control textual folds:
 *              - zo: open fold under cursor
 *              - zc: close fold under cursor
 *              - zr: open all folds in file
 *              - zm: close all folds in file
 * - Enable C syntax-highlighting if possible (In Vi[m], type ':se syntax=c')
 }}}*/
/*{{{ COMPILE FLAGS */

/* VIRTUAL_BOT:
 * ============
 * DEFINE...........if testing on PC
 * COMMENT-OUT......if loading to microcontroller (NOTE: Leave this compile flag alone! If you want to compile
 *                  and load this program to a microcontroller, run 'sh build.sh' in the containing directory
 *                  which -- among other build processes -- temporarily comments-out the VIRTUAL_BOT compile flag
 *                  and compiles the code to an executable file called 'AVRBot' to be used on a microcontroller.)
 */
#define VIRTUAL_BOT

/* DEBUG_GRID:
 * ===========
 * DEFINE...........if ASCII Grid should be sent to serial monitor while running on microcontroller
 * COMMENT-OUT......if not connected to Serial monitor while running on microcontroller (NOTE: Leave this compile
 *                  flag alone when testing on PC -- it won't affect anything.)
 */
#define DEBUG_GRID

/* ACTUATORS_ON:
 * ============
 * DEFINE...........if loading code to microcontroller and actuators on robot should be enabled for wheel motors,
 *                  robotic arm, etc. (all actuators)
 * COMMENT-OUT......if loading code to microcontroller and actuators on robot should *NOT* be enabled (NOTE: Leave
 *                  this compile flag alone when testing on PC -- it won't affect anything.)
 */
#define ACTUATORS_ON

/* REAR_FACING_RANGE_SENSORS:
 * ==========================
 * DEFINE...........if robot has distance sensors on its posterior
 * COMMENT-OUT......if otherwise
 */
/* #define REAR_FACING_RANGE_SENSORS */

/* LONG_RANGE_SENSORS:
 * ===================
 * DEFINE...........if robot has long-range distance-sensing capability (2 segments) FOR EACH SIDE it can sense
 *                  (e.g. if REAR_FACING_RANGE_SENSORS is defined along with LONG_RANGE_SENSORS, it is assumed
 *                  that the robot has long range sensors on its posterior)
 * COMMENT-OUT......if otherwise
 */
#define LONG_RANGE_SENSORS

/* REMEMBER_VISITED_NODES:
 * =======================
 * DEFINE...........if robot should keep track of which nodes (spaces) it has visited --
 *                  most, if not all, algorithms should only rely on segments (boundaries) to make decisions
 * COMMENT-OUT......if robot does NOT need to keep track of where it has been --
 *                  do this to save space and simplify run time when possible
 */
/* #define REMEMBER_VISITED_NODES */

/* DYNAMIC_SENSE:
 * ==============
 * DEFINE...........if robot should mark a segment as UNBLOCKED if it currently senses it as BLOCKED but it was
 *                  previously sensed as UNBLOCKED (meaning, it's probably the enemy robot instead of a
 *                  stationary obstacle and it may move out of the way) -- *** USE WITH EXTREME CARE ***
 *                  (UNDER POSSIBLE DEVELOPMENT)
 * COMMENT-OUT......for all intents and purposes
 */
/* #define SMART_SENSE */

/* }}} */
/*{{{ PLATFORM-DEPENDENT MACROS */

/* CLEAR and ERROR: */
/* ================ */
#include <stdlib.h>     /* malloc, free, exit */
#ifdef VIRTUAL_BOT          /* >>> PC test environment <<< */
    #include <stdio.h>
    #include <string.h> /* strncpy, strlen */
    #define ERROR(msg) fprintf(stderr, msg);
    #ifdef _WIN32
        #define CLEAR system("cls");
    #elif defined(__linux__) || defined(__APPLE__)
        #define CLEAR system("clear");
    #endif
#elif defined(DEBUG_GRID)   /* >>> microcontrollers connected to serial monitor <<< */
    #include <stdio.h>
    #define CLEAR printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    #define ERROR(msg) printf(msg);
#else                       /* >>> microcontrollers *NOT* connected to serial monitor <<< */
    #define CLEAR ;
    #define ERROR(msg) ; /* LL: add your error-message mechanism definition here
                                for when *NOT* connected to serial monitor */
#endif

/* Dynamic Virtual Grid Interaction */
/* ================================ */
#ifdef _WIN32
    /* TODO: Add Windows support for interactive console text */
#elif defined(__linux__) || defined(__APPLE__)
    #include <termios.h> /* termios, TCSANOW, ECHO, ICANON */
    #include <unistd.h>  /* STDIN_FILENO */
    struct termios oldt, newt;  /* terminal I/O settings */
#endif

/*}}}*/
/*{{{ DECLARATIONS */

/* Utilities: */
/* ========== */
#define abs(x) ((x) < 0 ? -1 * (x) : (x))
int getNumDigits(int);

/* Constants: */
/* ========== */
#define NUM_ROWS            7                       /* number of rows in grid-world */
#define NUM_COLS            6                       /* number of columns in grid-world */
#define NUM_NODES           (NUM_ROWS * NUM_COLS)   /* number of nodes in grid-world */
#define NUM_HORIZ_SEG_ROWS  (NUM_ROWS + 1)          /* number of rows of horizontal segments */
#define NUM_HORIZ_SEG_COLS  NUM_COLS                /* number of columns of horizontal segments  */
#define NUM_VERT_SEG_ROWS   NUM_ROWS                /* number of rows of vertical segments */
#define NUM_VERT_SEG_COLS   (NUM_COLS + 1)
#define NUM_DIRECTIONS      4

/* ADT's: */
/* ====== */
typedef enum
{
    FALSE = 0,
    TRUE = 1
} Bool;                         /* C does not have a primitive Bool type */
typedef enum
{
    UP = 0,     FRONT = 0,
    RIGHT = 1,
    DOWN = 2,   BACK = 2,
    LEFT = 3
} Direction;                    /* indicates absolute or relative direction */
typedef enum
{
    BLOCKED,
    UNBLOCKED,
    IDK
} Segment;                      /* indicates knowledge of presense of blocks */
#ifdef REMEMBER_VISITED_NODES
    typedef enum
    {
        UNVISITED,
        VISITED
    } Node;                     /* indicates whether or not a node has been visited */
#endif
    typedef struct
    {
        int row;        /* row of node */
        int col;        /* column of node */
        int motions;    /* number of motions ((moves + turns) it took to get to this node from start of path) */
        Direction dir;  /* direction that robot is facing when it enters this node */
    } Tile;                     /* used to return coordinates from functions */
    typedef struct _PathNode
    {
        Tile tile;
        struct _PathNode *next;
    } PathNode;                 /* linked list node for path home */
    typedef PathNode *Path;     /* linked list for path home */
    typedef struct _PathListNode
    {
        Path path;
        struct _PathListNode *next;
    } PathListNode;                 /* linked list node for collection of possible paths */
    typedef PathListNode *PathList; /* linked list for collection of possible paths */

/* Position and Orientation of Robot: */
/* ================================== */
int current[2];           /* [0]: current row, [1]: current column -- updates in moveForward() */
#define ROW 0
#define COL 1
Direction direction;      /* stores current direction -- updates in turn functions */

/* Environmental Information: */
/* ========================== */
Bool haveFlag;                            /* FALSE until we've picked up flag, then set to TRUE */
#ifdef REMEMBER_VISITED_NODES
    Node grid[NUM_ROWS][NUM_COLS];        /* 2D array comprised of every Node in grid; either UNVISITED or VISITED -- updates in moveForward() */
#endif
/* 2D array comprised of all horizontal Segments; BLOCKED, UNBLOCKED, or IDK -- updates in checkForBlocks() */
Segment horizSeg[NUM_HORIZ_SEG_ROWS][NUM_HORIZ_SEG_COLS];
/* 2D array comprised of all vertical Segments; BLOCKED, UNBLOCKED, or IDK -- updates in checkForBlocks() */
Segment vertSeg[NUM_VERT_SEG_ROWS][NUM_VERT_SEG_COLS];

/* Return Home Algorithm */
/* ===================== */
PathList pathsHome;                         /* collection of possible paths home */
int pathSize;                               /* length of paths in pathsHome */
PathListNode *pathListNodeContainingTile(Tile, PathList, int *);
void freePath(Path *);                      /* deallocates a Path */
void freePathList(PathList *);              /* deallocates a PathList (quad tree) */
int pathLength(Path);                       /* returns the length of a Path */
PathNode *lastPathNode(Path);               /* returns a pointer to the last node in a Path */
Tile relCoordinate(Direction, Tile);        /* returns Tile (i.e. node coordinate) in relative direction from a given Tile */
Segment segRelFrom(Direction, Tile);        /* returns status of segment adjacent to a Tile in relative Direction */
void returnHome(void);                      /* moves robot back to node (0, 0) in fastest possible way */
Path *copyPath(Path);                       /* returns a dynamically allocated copy of a Path */
Path *appendPathNode(Path, Tile);           /* returns a pointer to a new Path that is a copy of the first one with a Tile appended to it */
Bool expandFirstBranchInTree(PathList);     /* expands the first Path in the PathList argument and returns TRUE if found a path home */
Bool expandTreeOneLevel(PathList);          /* expands the PathList by extending each Path by one Tile in each possible direction */
PathList *pruneTree(PathList *);            /* removes old, redundant, and dead-end Paths from the PathList */
Path *branchAndBound(void);                 /* returns a pointer to the fastest path home (one with the fewest number of spaces) */
void returnHome(void);                      /* moves the robot to upper-left-most node in the grid as quickly as possible with known info */
Bool followPath(Path);                      /* moves along a specified path -- all Nodes in path must be adjacent with no blocks in the way */
#ifdef VIRTUAL_BOT
    char *DirectionToStr(Direction);
    void displayPath(Path);
    void displayPathList(PathList);
    int getTileIndex(int, int);
#endif
Path *fastestPath = NULL;                   /* pointer to fastest route home */
Bool segmentsChanged = FALSE;

#ifdef VIRTUAL_BOT
/* Virtual Objects -- VIRTUAL GRID ONLY */
/* ==================================== */
    /* The following 2 arrays are representations of locations of physical obstacles, which may or may not
       have been sensed by the robot. Up to 13 segments will be blocked based on competition specification of blocks. */
    Segment blockedHorizSeg[NUM_HORIZ_SEG_ROWS][NUM_HORIZ_SEG_COLS];
    Segment blockedVertSeg[NUM_VERT_SEG_ROWS][NUM_VERT_SEG_COLS];
    int enemy[2];   /* [0]: current row of enemy robot, [1]: current column of enemy robot -- updates in moveEnemyRobot() */
#endif

/* Initialization and Deallocation: */
/* ================================ */
#ifdef VIRTUAL_BOT
    void initializeTestVariables(int, char **); /* initializes variables used only in virtual grid */
    void loadTest(void);                        /* loads a test file */
    void saveTest(void);                        /* saves a test file */
    void closeTest(void);                       /* closes a test file */
    FILE *testFile;
    #define FILE_NAME_LENGTH 256
    char fileName[FILE_NAME_LENGTH];
#endif
void initializeGlobalVariables(void);   /* INITIALIZE: haveFlag, direction, current, grid, horizSeg, vertSeg */
void initZigZagRoute(void);             /* assign row and column numbers to each Node in route, in a DOWN-RIGHT-UP-RIGHT-DOWN(...) zig-zag pattern */
void deallocateMemory(void);            /* deallocate memory for dynamic global variables */

/* Actuators: */
/* ========== */
void turnAbs(Direction);    /* turns robot in absolute Direction specified by input argument */
void turnLeft(void);        /* turns robot 90 degrees to relative LEFT */
void turnRight(void);       /* turns robot 90 degrees to relative RIGHT */
void turn180(void);         /* turns robot 180 degrees in relative orientation */
void moveForward(void);     /* checks to see if segment in relative FRONT is UNBLOCKED, and then moves to that node */
void captureFlag(void);     /* captures the flag (makes virtual robot just beep a few times) */

/* Patterned Movement: */
/* =================== */
void moveToFlag(void);      /* algorithm for getting to the flag */
void wallHugUntil(Direction, Bool(*)(void *), void *);
#ifdef REMEMBER_VISITED_NODES
/* TODO: Add appropriate functions here as needed... */
#endif

/* Sensors: */
/* ======== */
void checkForBlocks(void);
Bool left_s(void);           /* for blocks to the left */
Bool right_s(void);          /* for blocks to the right */
Bool front_s(void);          /* for blocks to the front-left */
#ifdef REAR_FACING_RANGE_SENSORS
    Bool back_s(void);       /* for blocks to the back */
#endif
#ifdef LONG_RANGE_SENSORS
    Bool left2_s(void);      /* for blocks to the left 2 spaces out */
    Bool right2_s(void);     /* for blocks to the right 2 spaces out */
    Bool front2_s(void);     /* for blocks to the front-left 2 spaces out */
    #ifdef REAR_FACING_RANGE_SENSORS
        Bool back2_s(void);  /* for blocks to the back 2 spaces out */
    #endif
#endif
Bool sense(Direction, short);     /* calls sensor function in relative direction at a specified number of segments away */
/* Robot Memory Check Test Functions */
/* ================================= */
Segment segAbs(Direction);              /* returns status of adjacent segment, absolute Direction */
Segment segAbsx(Direction, int);        /* returns status of x away segment, absolute Direction */
Segment segRel(Direction);              /* returns status of adjacent segment, relative Direction */

#ifdef VIRTUAL_BOT
    /* Console Output Test Functions */
    /* ============================= */
    unsigned short rowNumWidth;
    void displayGrid(void);         /* virtual grid */
    void waitForEnter(void);        /* called in pauseAndClear; also a debugging tool */
    void pauseAndClear(void);       /* used in displayGrid */
    int getline(char *, int);       /* reads a line, returns length */
    /* Virtual Object Presence Functions */
    /* ================================= */
    Segment blockedSegAbs(Direction);   /* indicates presence of virtual grid block on segment adjacent to current Node; absolute Direction */
    Segment blockedSegRel(Direction);   /* indicates presence of virtual grid block on segment adjacent to current Node; relative Direction */
    Bool enemyPresenceAbs(Direction);   /* indicates presence of virtual enemy robot in Node adjacent to current Node; absolute Direction */
    Bool enemyPresenceRel(Direction);   /* indicates presence of virtual enemy robot in Node adjacent to current Node; relative Direction */
    #ifdef LONG_RANGE_SENSORS
        Segment blockedSegAbs2(Direction);   /* same as blockedSegAbs, except 2 segments down instead of 1 */
        Segment blockedSegRel2(Direction);   /* same as blockedSegRel, except 2 segments down instead of 1 */
        Bool enemyPresenceAbs2(Direction);   /* same as enemyPresenceAbs, except 2 Nodes down instead of 1 */
        Bool enemyPresenceRel2(Direction);   /* same as enemyPresenceRel, except 2 Nodes down instead of 1 */
    #endif
    /* Dynamic Virtual Grid Interaction */
    /* ================================ */
    int buffer; /* temporarily holds one character from stdin, including EOF, which is an integer */
    int cursor[2] = {1, 0};                     /* cursor for test setup */
    enum { HORIZ, VERT } cursorMode = HORIZ;    /* whether cursor is currently selecting horizSegs or vertSegs */
    Bool setupMode = TRUE;
    /* #defines are used to mimic inline functions, which do not exist in ANSI C */
    #define ctrlUp(c)               (c == 'w'  || c == 'k')
    #define ctrlDown(c)             (c == 's'  || c == 'j')
    #define ctrlLeft(c)             (c == 'a'  || c == 'h')
    #define ctrlRight(c)            (c == 'd'  || c == 'l')
    #define ctrlDone(c)             (c == 'q'  || c == EOF)
    #define ctrlToggleVertHoriz(c)  (c == 'f'  || c == 'n')
    #define ctrlToggleBlock(c)      (c == '\n' || c == ' ')
    #define ctrlLoadFile(c)         (c == 'o')
    #define ctrlSaveFile(c)         (c == '[')
    #define ctrlSaveAsFile(c)       (c == ']')
    #define advanceRobot(c)         (c == '\n' || c == ' ')
    void controlEnemyRobot(void);
    void moveEnemyRobot(Direction);
    void moveCursor(Direction);
    void toggleBlock();
    void toggleVertHoriz();
    void initializeTestControls(void);
    void tearDownTestControls(void);
#elif defined(DEBUG_GRID)
    void displayDebugGrid(void);    /* grid to be displayed on serial monitor when connected to microcontroller */
#endif

/*}}}*/
/*{{{ MAIN ROUTINE */

#ifdef VIRTUAL_BOT
int main(int argc, char **argv)
#else
int main(void)  /* LL: Add any parameters you wish */
#endif
{
    initializeGlobalVariables();            /* haveFlag, direction, current, grid, horizSeg, vertSeg, route */

#ifdef VIRTUAL_BOT
    initializeTestControls();
    initializeTestVariables(argc, argv);    /* blockedHorizSeg, blockedVertSeg */
#endif
    checkForBlocks();   /* check for blocks surrounding start space -- MANDATORY */
    moveToFlag();
    returnHome();

#ifdef VIRTUAL_BOT
    tearDownTestControls();
#endif

    /* this always comes before the return statement */
    deallocateMemory();

    return 0;
}

/*}}}*/
/*{{{ INITIALIZATION AND DEALLOCATION FUNCTIONS */

/* returns the number of digits in a number n */
int getNumDigits(int n)
{
    if (n / 10 == 0) /* make good use of integer division... */
        return 1;
    else
        return 1 + getNumDigits(n / 10); /* ... and recursion */
}

/*  FUNCTION: initializeGlobalVariables
    initializes all global variables common to both the actual robot and virtual grid:

    Variable:       Meaning:
    =========       ========
    haveFlag        FALSE until the robot captures the flag, in which case it is set to TRUE in moveForward()
    direction       Direction that robot is currently facing -- updated in turn functions
    current[ROW]    Number of row currently occupied by robot -- updated in moveForward()
    current[COL]    Number of column currently occupied by robot -- updated in moveForward()
    grid[][]        2D array of Nodes, either VISITED or UNVISITED
    horizSeg[][]    2D arrays of Segments, either UNBLOCKED, BLOCKED, or IDK -- updated in checkForBlocks,
    vertSeg[][]     which is called in moveForward()
    route           Planned array of Nodes to be traveled sequentially by robot
*/
void initializeGlobalVariables(void)
{
    int i, j;   /* loop control variables */

    /* haven't found finish space yet */
    haveFlag = FALSE;

    /* current orientation and position of robot */
    direction = DOWN;
    current[ROW] = 0; current[COL] = 0;

#ifdef REMEMBER_VISITED_NODES
    /* initialize all grid Nodes to UNVISITED... */
    for (i = 0; i < NUM_ROWS; ++i)
    {
        for (j = 0; j < NUM_COLS; ++j)
            grid[i][j] = UNVISITED;
    }
    /* ...except for the start space */
    grid[0][0] = VISITED;
#endif

    /* initialize horizontal segments */
    for (i = 0; i < NUM_HORIZ_SEG_ROWS; ++i)    /* set all horizontal segments to IDK */
    {
        for (j = 0; j < NUM_HORIZ_SEG_COLS; ++j)
            horizSeg[i][j] = IDK;
    }
    for (i = 0; i < NUM_HORIZ_SEG_COLS; ++i)     /* set border horizontal segments to BLOCKED */
    {
        horizSeg[0][i] = BLOCKED;
        horizSeg[NUM_HORIZ_SEG_ROWS - 1][i] = BLOCKED;
    }

    /* initialize vertical segments */
    for (i = 0; i < NUM_VERT_SEG_ROWS; ++i)     /* set all vertical segments to IDK */
    {
        for (j = 0; j < NUM_VERT_SEG_COLS; ++j)
            vertSeg[i][j] = IDK;
    }
    for (i = 0; i < NUM_VERT_SEG_ROWS; ++i)     /* set border vertical segments to BLOCKED */
    {
        vertSeg[i][0] = BLOCKED;
        vertSeg[i][NUM_VERT_SEG_COLS - 1] = BLOCKED;
    }

    return;
}

#ifdef VIRTUAL_BOT
/*  FUNCTION: initializeTestVariables
    initializes all global variables used only in virtual grid:

    Variable:           Meaning:
    =========           ========
    blockedHorizSeg[][] 2D arrays containing virtual representations of either the absense or presense of physical blocks
    blockedVertSeg[][]      on each Segment in the grid
*/
void initializeTestVariables(int argc, char **argv)
{
    int i, j;                    /* LCV's */

    /* width of row number to be printed in displayGrid */
    rowNumWidth = getNumDigits(NUM_ROWS);

    /* initialize all PHYSICAL (independent of robot) segments to UNBLOCKED */
    for (i = 0; i < NUM_HORIZ_SEG_ROWS; ++i)
    {
        for (j = 0; j < NUM_HORIZ_SEG_COLS; ++j)
            blockedHorizSeg[i][j] = UNBLOCKED;
    }
    for (i = 0; i < NUM_VERT_SEG_ROWS; ++i)
    {
        for (j = 0; j < NUM_VERT_SEG_COLS; ++j)
            blockedVertSeg[i][j] = UNBLOCKED;
    }

    /* initialize position of enemy robot -- always starts in far corner */
    enemy[ROW] = NUM_ROWS - 1;
    enemy[COL] = NUM_COLS - 1;

    if (argc > 1)
    {
        strncpy(fileName, argv[1], FILE_NAME_LENGTH);
        loadTest();
    }
    else
    {
        /* set locations of physical blocks*/
        do {
            displayGrid();
            buffer = getchar();
            if (ctrlToggleVertHoriz(buffer))
                toggleVertHoriz();
            else if (ctrlToggleBlock(buffer))
                toggleBlock();
            else if (ctrlUp(buffer))
                moveCursor(UP);
            else if (ctrlDown(buffer))
                moveCursor(DOWN);
            else if (ctrlLeft(buffer))
                moveCursor(LEFT);
            else if (ctrlRight(buffer))
                moveCursor(RIGHT);
            else if (ctrlLoadFile(buffer))
            {
                printf("\bEnter file name: ");
                getline(fileName, FILE_NAME_LENGTH);
                loadTest();
            }
            else if (ctrlSaveFile(buffer))
            {
                saveTest();
            }
            else if (ctrlSaveAsFile(buffer))
            {
                printf("\bEnter file name: ");
                getline(fileName, FILE_NAME_LENGTH);
                saveTest();
            }
        } while (!ctrlDone(buffer));
    }
    setupMode = FALSE;
    cursor[ROW] = cursor[COL] = -1; /* move cursor off of grid */
    CLEAR
    controlEnemyRobot();

    return;
}

/* loads a test file */
void loadTest(void)
{
    char tmpBuf[256];
    testFile = fopen(fileName, "r");
    if (!testFile)
        fprintf(stderr, "Cannot open %s\n", fileName);
    else
    {
        /* TODO: Read contents of testFile into blockedHorizSeg and blockedVertSeg */
        scanf("%s", tmpBuf);
        printf("%s\n", tmpBuf);
        pauseAndClear();

        closeTest();
    }

    return;
}

/* saves a test file */
void saveTest(void)
{
    testFile = fopen(fileName, "w");
    if (!testFile)
        fprintf(stderr, "Cannot open %s\n", fileName);
    else
    {
        /* TODO: Write contents of blockedHorizSeg and blockedVertSeg into testFile */
        closeTest();
    }

    return;
}

/* closes a test file */
void closeTest(void)
{
    fclose(testFile);
    if (ferror(stdout))
    {
        ERROR("error writing stdout\n")
        exit(1);
    }

    return;
}

#endif

/* deallocates dynamically allocated memory for global variables */
void deallocateMemory(void)
{
    /* TODO: Add proper deallocation as needed */
    return;
}

/*}}}*/
/*{{{ SENSOR FUNCTIONS */

/* calls the appropriate sensor function in direction dir, one or two segments away */
Bool sense(Direction dir, short oneOrTwo)
{
    if (oneOrTwo == 1)
        switch (dir)
        {
            case FRONT: return front_s(); break;
            case  LEFT: return  left_s(); break;
            case RIGHT: return right_s(); break;
#ifdef REAR_FACING_RANGE_SENSORS
            case  BACK: return  back_s(); break;
#endif
            default:    ERROR("Misuse of function sense\n") break;
        }
#ifdef LONG_RANGE_SENSORS
    else if (oneOrTwo == 2)
        switch (dir)
        {
            case FRONT: return front2_s(); break;
            case  LEFT: return  left2_s(); break;
            case RIGHT: return right2_s(); break;
#ifdef REAR_FACING_RANGE_SENSORS
            case  BACK: return  back2_s(); break;
#endif
            default:    ERROR("Misuse of function sense\n") break;
        }
#endif
    else
        ERROR("Misuse of function sense\n")

    return TRUE; /* dummy value */
}

/* call sensor functions to see if segments are blocked or not */
void checkForBlocks(void)
{
    int i, dir;

    /* define aliases for adjacent segments */
    Segment *segPtr[NUM_DIRECTIONS];
    #ifdef LONG_RANGE_SENSORS
        Segment *segPtr2[NUM_DIRECTIONS];
    #endif
    segPtr[UP]    = current[ROW] == 0            ? NULL : &(horizSeg[current[ROW]][current[COL]]);
    segPtr[DOWN]  = current[ROW] == NUM_ROWS - 1 ? NULL : &(horizSeg[current[ROW] + 1][current[COL]]);
    segPtr[LEFT]  = current[COL] == 0            ? NULL : &(vertSeg[current[ROW]][current[COL]]);
    segPtr[RIGHT] = current[COL] == NUM_COLS - 1 ? NULL : &(vertSeg[current[ROW]][current[COL] + 1]);
    #ifdef LONG_RANGE_SENSORS
        segPtr2[UP]    = current[ROW] < 2            ? NULL : &(horizSeg[current[ROW] - 1][current[COL]]);
        segPtr2[DOWN]  = current[ROW] > NUM_ROWS - 3 ? NULL : &(horizSeg[current[ROW] + 2][current[COL]]);
        segPtr2[LEFT]  = current[COL] < 2            ? NULL : &(vertSeg[current[ROW]][current[COL] - 1]);
        segPtr2[RIGHT] = current[COL] > NUM_COLS - 3 ? NULL : &(vertSeg[current[ROW]][current[COL] + 2]);
    #endif
    for (i = 0; i < NUM_DIRECTIONS; ++i)
    {
        dir = (direction + i) % NUM_DIRECTIONS;
    #ifndef REAR_FACING_RANGE_SENSORS
        if (i != BACK)
    #endif
        {
            if (segPtr[dir])
                *segPtr[dir] = sense((Direction)i, 1) ? BLOCKED : UNBLOCKED;
            #ifdef LONG_RANGE_SENSORS
            if (segPtr2[dir] && segPtr[dir] && *segPtr[dir] == UNBLOCKED)
                *segPtr2[dir] = (sense((Direction)i, 2) ? BLOCKED : UNBLOCKED);
            #endif
        }
    }

    return;
}

/* TO SENSE BLOCKS: TRUE -- block is sensed in adjacent segment */
Bool left_s(void)   /* for blocks to the left */
{
#ifdef VIRTUAL_BOT
    return (blockedSegRel(LEFT) == BLOCKED || enemyPresenceRel(LEFT)) ? TRUE : FALSE;
#else
    /* LL: code goes here for sensing segment to the left */
    return TRUE; /* dummy code; LL: remove this line when finished! */
#endif
}

Bool right_s(void)  /* for blocks to the right */
{
#ifdef VIRTUAL_BOT
    return (blockedSegRel(RIGHT) == BLOCKED || enemyPresenceRel(RIGHT)) ? TRUE : FALSE;
#else
    /* LL: code goes here for sensing segment to the right */
    return TRUE; /* dummy code; LL: remove this line when finished! */
#endif
}

Bool front_s(void)  /* for blocks to the front */
{
#ifdef VIRTUAL_BOT
    return (blockedSegRel(FRONT) == BLOCKED || enemyPresenceRel(FRONT) == TRUE) ? TRUE : FALSE;
#else
    /* LL: code goes here for sensing segment to the front */
    return TRUE; /* dummy code; LL: remove this line when finished! */
#endif
}

#ifdef REAR_FACING_RANGE_SENSORS
Bool back_s(void)   /* for blocks to the front */
{
#ifdef VIRTUAL_BOT
    return (blockedSegRel(BACK) == BLOCKED || enemyPresenceRel(BACK)) ? TRUE : FALSE;
#else
    /* LL: code goes here for sensing segment to the back */
    return TRUE; /* dummy code; LL: remove this line when finished! */
#endif
}
#endif

#ifdef LONG_RANGE_SENSORS
    /* TO SENSE BLOCKS: TRUE -- block is sensed in segment two to the left */
    Bool left2_s(void)     /* for blocks to the left */
    {
    #ifdef VIRTUAL_BOT
        return (blockedSegRel2(LEFT) == BLOCKED || enemyPresenceRel2(LEFT)) ? TRUE : FALSE;
    #else
        /* LL: code goes here for sensing two segments to the left */
        return TRUE; /* dummy code; LL: remove this line when finished! */
    #endif
    }

    Bool right2_s(void)    /* for blocks to the right */
    {
    #ifdef VIRTUAL_BOT
        return (blockedSegRel2(RIGHT) == BLOCKED || enemyPresenceRel2(RIGHT)) ? TRUE : FALSE;
    #else
        /* LL: code goes here for sensing two segments to the right */
        return TRUE; /* dummy code; LL: remove this line when finished! */
    #endif
    }

    Bool front2_s(void)   /* for blocks to the front */
    {
    #ifdef VIRTUAL_BOT
        return (blockedSegRel2(FRONT) == BLOCKED || enemyPresenceRel2(FRONT)) ? TRUE : FALSE;
    #else
        /* LL: code goes here for sensing two segments to the front */
        return TRUE; /* dummy code; LL: remove this line when finished! */
    #endif
    }

    #ifdef REAR_FACING_RANGE_SENSORS
    Bool back2_s(void)   /* for blocks to the front */
    {
    #ifdef VIRTUAL_BOT
        return (blockedSegRel2(BACK) == BLOCKED || enemyPresenceRel2(BACK)) ? TRUE : FALSE;
    #else
        /* LL: code goes here for sensing two segments to the back */
        return TRUE; /* dummy code; LL: remove this line when finished! */
    #endif
    }
    #endif
#endif

/*}}}*/
/*{{{ ACTUATOR FUNCTIONS */

/* turn robot to face specified absolute Direction */
void turnAbs(Direction dirToFace)
{
    /* want to face up */
    if (dirToFace == UP)
    {
        if (direction == DOWN)
            turn180();
        else if (direction == LEFT)
            turnRight();
        else if (direction == RIGHT)
            turnLeft();
        /* else (direction == UP) ---> no need to turn */
    }
    /* want to face down */
    else if (dirToFace == DOWN)
    {
        if (direction == UP)
            turn180();
        else if (direction == LEFT)
            turnLeft();
        else if (direction == RIGHT)
            turnRight();
        /* else (direction == DOWN) ---> no need to turn */
    }
    /* want to face left */
    else if (dirToFace == LEFT)
    {
        if (direction == UP)
            turnLeft();
        else if (direction == DOWN)
            turnRight();
        else if (direction == RIGHT)
            turn180();
        /* else (direction == LEFT) ---> no need to turn */
    }
    /* want to face right */
    else /* dirToFace == RIGHT */
    {
        if (direction == UP)
            turnRight();
        else if (direction == DOWN)
            turnLeft();
        else if (direction == LEFT)
            turn180();
        /* else (direction == RIGHT) ---> no need to turn */
    }

  return;
}

/* turns robot 90 degrees to relative LEFT */
void turnLeft(void)
{
#if !defined(VIRTUAL_BOT) && defined(ACTUATORS_ON)
    /* LL: stepper motor code for turning robot 90 degrees to the left goes here */
#endif

    if (direction == UP)
        direction = LEFT;
    else if (direction == DOWN)
        direction = RIGHT;
    else if (direction == LEFT)
        direction = DOWN;
    else /* direction == RIGHT */
        direction = UP;

    return;
}

/* turns robot 90 degrees to relative RIGHT */
void turnRight(void)
{
#if !defined(VIRTUAL_BOT) && defined(ACTUATORS_ON)
    /* LL: stepper motor code for turning robot 90 degrees to the right goes here */
#endif

    if (direction == UP)
        direction = RIGHT;
    else if (direction == DOWN)
        direction = LEFT;
    else if (direction == LEFT)
        direction = UP;
    else /* direction == RIGHT */
        direction = DOWN;

    return;
}

/* turns robot 180 degrees from it's current relative position */
void turn180(void)
{
#if !defined(VIRTUAL_BOT) && defined(ACTUATORS_ON)
    /* LL: stepper motor code for turning robot 180 degrees goes here */
#endif

    if (direction == UP)
        direction = DOWN;
    else if (direction == DOWN)
        direction = UP;
    else if (direction == LEFT)
        direction = RIGHT;
    else /* direction == RIGHT */
        direction = LEFT;

    return;
}

/* if the relative FRONT Segment is UNBLOCKED (no blocks and not on edge of grid), move forward one Node; then check for surrounding
   blocks and update the grid and Segment arrays; then capture the flag if at flag--if capturing flag, update the global haveFlag variable */
void moveForward(void)
{
    /* define aliases for adjacent segments -- used when returning home to see if status of Segments have changed */
    Segment top, btm, lft, rgt;
    #ifdef LONG_RANGE_SENSORS
        Segment top2, btm2, lft2, rgt2;
    #endif

    /* check to make sure the robot can make the movement */
    if (segRel(FRONT) == UNBLOCKED)
    {
        #if !defined(VIRTUAL_BOT) && defined(ACTUATORS_ON)
            /* LL: stepper motor code for moving robot one space forward goes here
             *     CRITICAL: Robot must move entirely to next space before next lines of code execute!!! */
        #endif

        /* update current array */
        if (direction == UP)
            --current[ROW]; /* decrement row */
        else if (direction == DOWN)
            ++current[ROW]; /* increment row */
        else if (direction  == LEFT)
            --current[COL]; /* decrement column */
        else /* direction == RIGHT */
            ++current[COL]; /* increment column */

        #ifdef REMEMBER_VISITED_NODES
            grid[current[ROW]][current[COL]] = VISITED;
        #endif

        /* if in bottom-right corner, capture the flag and set haveFlag to TRUE */
        if (current[ROW] == NUM_ROWS - 1 && current[COL] == NUM_COLS - 1)
        {
            captureFlag();
            haveFlag = TRUE;
        }
    }
    else
    {
        ERROR("Logic Error: Planned to move forward, but FRONT is blocked\n")
    }

    #ifdef VIRTUAL_BOT
        controlEnemyRobot();
    #elif defined(DEBUG_GRID)
        displayDebugGrid();
    #endif

    /* if robot is on its way home */
    if (fastestPath)
    {
        top = horizSeg[current[ROW]][current[COL]];
        btm = horizSeg[current[ROW] + 1][current[COL]];
        lft = vertSeg[current[ROW]][current[COL]];
        rgt = vertSeg[current[ROW]][current[COL] + 1];
        #ifdef LONG_RANGE_SENSORS
            top2 = current[ROW] < 1            ? -1 : horizSeg[current[ROW] - 1][current[COL]];
            btm2 = current[ROW] > NUM_ROWS - 2 ? -1 : horizSeg[current[ROW] + 2][current[COL]];
            lft2 = current[COL] < 1            ? -1 : vertSeg[current[ROW]][current[COL] - 1];
            rgt2 = current[COL] > NUM_COLS - 2 ? -1 : vertSeg[current[ROW]][current[COL] + 2];
        #endif
    }

    /* always check for blocks */
    checkForBlocks();

    if (fastestPath)
    {
        if (  (top != horizSeg[current[ROW]][current[COL]]    )
           || (btm != horizSeg[current[ROW] + 1][current[COL]])
           || (lft != vertSeg[current[ROW]][current[COL]]     )
           || (rgt != vertSeg[current[ROW]][current[COL] + 1] )
        #ifdef LONG_RANGE_SENSORS
           || (top2 >= 0 && top2 != horizSeg[current[ROW] - 1][current[COL]])
           || (btm2 >= 0 && btm2 != horizSeg[current[ROW] + 2][current[COL]])
           || (lft2 >= 0 && lft2 != vertSeg[current[ROW]][current[COL] - 1] )
           || (rgt2 >= 0 && rgt2 != vertSeg[current[ROW]][current[COL] + 2] )
        #endif
           ) segmentsChanged = TRUE;
        else segmentsChanged = FALSE;
    }

    return;
}

/* captures the flag (makes virtual robot just beep a few times) */
void captureFlag(void)
{
    #ifdef VIRTUAL_BOT
        printf("\a\a\a"); /* beep a few times */
    #elif defined(ACTUATORS_ON)
        /* LL: capture the flag code */
    #endif
    return;
}

/*}}}*/
/*{{{ ROBOT MEMORY CHECK FUNCTIONS */

/* returns status of adjacent segment, specified by absolute Direction */
Segment segAbs(Direction absDir)
{
    return segAbsx(absDir, 0);
}

/* returns status of x away segment, specified by absolute Direction */
Segment segAbsx(Direction absDir, int x)
{
    if (absDir == UP)
        return horizSeg[current[ROW] - x][current[COL]];
    else if (absDir == DOWN)
        return horizSeg[current[ROW] + 1 + x][current[COL]];
    else if (absDir == LEFT)
        return vertSeg[current[ROW]][current[COL] - x];
    else /* absDir == RIGHT */
        return vertSeg[current[ROW]][current[COL] + 1 + x];
}

/* returns status of adjacent segment, specified by relative Direction */
Segment segRel(Direction relSeg)
{
    Segment returnValue;

    /* currently facing absolute UP */
    if (direction == UP)
    {
        returnValue = segAbs(relSeg); /* same as segAbs function */
    }
    /* currently facing absolute DOWN */
    else if (direction == DOWN)
    {
        if (relSeg == FRONT) /* absolute DOWN */
            returnValue = segAbs(DOWN);
        else if (relSeg == BACK) /* absolute UP */
            returnValue = segAbs(UP);
        else if (relSeg == LEFT) /* absolute RIGHT */
            returnValue = segAbs(RIGHT);
        else /* relSeg == RIGHT ---> absolute LEFT */
            returnValue = segAbs(LEFT);
    }
    /* currently facing absolute LEFT */
    else if (direction == LEFT)
    {
        if (relSeg == FRONT) /* absolute LEFT */
            returnValue = segAbs(LEFT);
        else if (relSeg == BACK) /* absolute RIGHT */
            returnValue = segAbs(RIGHT);
        else if (relSeg == LEFT) /* absolute DOWN */
            returnValue = segAbs(DOWN);
        else /* relSeg == RIGHT ---> absolute UP */
            returnValue = segAbs(UP);
    }
    /* currently facing absolute RIGHT */
    else /* direction == RIGHT */
    {
        if (relSeg == FRONT) /* absolute RIGHT */
            returnValue = segAbs(RIGHT);
        else if (relSeg == BACK) /* absolute LEFT */
            returnValue = segAbs(LEFT);
        else if (relSeg == LEFT) /* absolute UP */
            returnValue = segAbs(UP);
        else /* relSeg == RIGHT ---> absolute DOWN */
            returnValue = segAbs(DOWN);
    }

    return returnValue;
}

/*}}}*/
/*{{{ PATTERNED MOVEMENT FUNCTIONS */

Bool inTopRow_OR_OpeningToRightUpAbove(void *startingRow)
{
    return current[ROW] == 0 || (current[ROW] < *(int *)startingRow && segAbs(RIGHT) == UNBLOCKED);
}

Bool inBottomRow_OR_OpeningToRight(void *dummy)
{
    return current[ROW] == NUM_ROWS - 1 || segAbs(RIGHT) == UNBLOCKED;
}

Bool inLeftCol_OR_OpeningBelowToLeft(void *startingCol)
{
    return current[COL] == 0 || (current[COL] < *(int *)startingCol && segAbs(DOWN) == UNBLOCKED);
}

Bool inRightCol_OR_OpeningBelow(void *dummy)
{
    return current[COL] == NUM_COLS - 1 || segAbs(DOWN) == UNBLOCKED;
}

/* moves the robot from the top-left space to the bottom-right in the grid */
void moveToFlag(void)
{
    int tmp;                       /* temporary storage for row and column number */
    int troubleSpot[2] = { 0, 0 }; /* location where we are blocked to both right and bottom */
    Bool stuck = FALSE;            /* indicates a failure of first attempt to get unstuck */

    /* while not in destination */
    while (!(current[ROW] == NUM_ROWS - 1 && current[COL] == NUM_COLS - 1))
    {
        /* we can move in the desired direction */
        if (segAbs(DOWN) == UNBLOCKED || segAbs(RIGHT) == UNBLOCKED)
        {
Down_Label:
            /* if able, move abs down until there is a block */
            if (segAbs(DOWN) == UNBLOCKED)
            {
                turnAbs(DOWN);
                do moveForward(); while (segAbs(DOWN) == UNBLOCKED);
            }
Right_Label:
            /* if able, move abs right until there is a block */
            if (segAbs(RIGHT) == UNBLOCKED)
            {
                turnAbs(RIGHT);
                do moveForward(); while (segAbs(RIGHT) == UNBLOCKED);
            }
        }
        /* if we can, we need to move either UP or LEFT */
        else if (segAbs(UP) == UNBLOCKED || segAbs(LEFT) == UNBLOCKED)
        {
            /* see if our current grid position is a trouble spot */
            /* TODO: do something with this "stuck" variable */
            if (troubleSpot[ROW] == current[ROW] && troubleSpot[COL] == current[COL])
            {
                stuck = TRUE;
            }
            else
            {
                /* save our current coordinates */
                troubleSpot[ROW] = current[ROW];
                troubleSpot[COL] = current[COL];
                stuck = FALSE;
            }

            /* we want to move in the direction that places us more in the center of the grid */
            /* in this case, moving up gets us closer to center (or we have no other choice) */
            if ((NUM_ROWS - current[ROW] <= NUM_COLS - current[COL] && segAbs(UP) == UNBLOCKED)
                 || segAbs(LEFT) == BLOCKED) /* no other choice */
            {
                /* move up until we can't go any further or there is an opening to the right */
                turnAbs(UP);
                do moveForward(); while (
                        segAbs(UP) == UNBLOCKED  &&
                        segAbs(RIGHT) == BLOCKED
                    );
                /* if we've found an opening to the right, take it */
                if (segAbs(RIGHT) == UNBLOCKED)
                    goto Right_Label;
                /* we are facing up and there are blocks to the front and right of us */
                else
                {
                    /* if we are not in the top row, we can wall hug right */
                    if (current[ROW] > 0)
                    {
                        tmp = current[ROW];
                        /* wall hug until in top row or there is an opening to the right in
                         * a row further up in the grid than the current node's row */
                        wallHugUntil(RIGHT, inTopRow_OR_OpeningToRightUpAbove, (void *)&tmp);
                        /* we've found an opening */
                        if (segAbs(RIGHT) == UNBLOCKED)
                            goto Right_Label;
                    }
                    /* we are in the top row and the right of us is blocked */
                    if (current[ROW] == 0)
                    {
                        turnAbs(DOWN);
                        /* move down until you reach the bottom or find an opening */
                        wallHugUntil(LEFT, inBottomRow_OR_OpeningToRight, NULL);
                        /* we've found an opening */
                        if (segAbs(RIGHT) == UNBLOCKED)
                            goto Right_Label;
                    }
                }
            }
            /* in this case, moving left gets us closer to center (or we have no other choice) */
            else
            {
                /* move left until we can't go any further or there is an opening to abs down */
                turnAbs(LEFT);
                do moveForward(); while (
                        segAbs(LEFT) == UNBLOCKED  &&
                        segAbs(DOWN) == BLOCKED
                    );
                /* if we've found an opening to abs down, take it */
                if (segAbs(DOWN) == UNBLOCKED)
                    goto Down_Label;
                /* we are facing left and there is a block in front of us */
                else
                {
                    /* if we are not in the left-most column, we can wall hug left */
                    if (current[COL] > 0)
                    {
                        tmp = current[COL];
                        /* wall hug until in left-most column or there is an opening to the abs down
                         * in a column further left in the grid than the current node's column */
                        wallHugUntil(LEFT, inLeftCol_OR_OpeningBelowToLeft, (void *)&tmp);
                        /* we've found an opening */
                        if (segAbs(DOWN) == UNBLOCKED)
                            goto Down_Label;
                    }
                    /* we are in the left-most column and below us is blocked */
                    if (current[COL] == 0)
                    {
                        turnAbs(RIGHT);
                        /* move right until you reach the far right or find an opening */
                        wallHugUntil(RIGHT, inRightCol_OR_OpeningBelow, NULL);
                        /* we've found an opening */
                        if (segAbs(DOWN) == UNBLOCKED)
                            goto Down_Label;
                    }
                }
            }
        }
        /* there is nothing open, so we need to sit still and read sensors until there is an opening */
        else
        {
            checkForBlocks();
#ifdef VIRTUAL_BOT
            controlEnemyRobot();
#endif
        }
    }
    return;
}

/* moves along a wall to relative LEFT or relative RIGHT until condition is met */
void wallHugUntil(Direction dir, Bool(*condition)(void *), void *args)
{
    if (dir == LEFT || dir == RIGHT)
        while (!(*condition)(args))
        {
            if (segRel(dir) == UNBLOCKED)
            {
                if (dir == LEFT)
                    turnLeft();
                else
                    turnRight();
            }
            else if (segRel(FRONT) == BLOCKED)
            {
                if (dir == LEFT)
                    turnRight();
                else
                    turnLeft();
            }
            moveForward();
        }
    else
        ERROR("wallHugUntil should only be sent LEFT or RIGHT as an argument\n")

    return;
}

/*}}}*/
/*{{{ RETURN HOME FUNCTIONS */
PathListNode *pathListNodeContainingTile(Tile t, PathList l, int *out)
{
    static Path p;
    if (!l) /* no paths in l */
    {
        out = NULL;
        return NULL;
    }
    p = l->path;
    while (p)       /* check first path in l for c */
    {
        if (t.row == p->tile.row && t.col == p->tile.col)
        {
            *out = p->tile.motions;  /* number of motions to get to existing node */
            return l;               /* pointer to PathListNode with path containing Tile t */
        }
        else
            p = p->next;
    }
    return pathListNodeContainingTile(t, l->next, out);
}

Tile *tileInPathList(Tile c, PathList l)
{
    Path p;
    if (!l) /* no paths in l */
        return NULL;
    p = l->path;    /* first path in l */
    while (p)       /* check first path in l for c */
    {
        if (c.row == p->tile.row && c.col == p->tile.col)
            return &p->tile;
        else
            p = p->next;
    }
    return tileInPathList(c, l->next);
}
void freePath(Path *p)
{
    PathNode *pathNodePtr = *p;
    while (*p)
    {
        *p = (*p)->next;
        free(pathNodePtr);
        pathNodePtr = *p;
    }
    return;
}
void freePathList(PathList *pl)
{
    PathListNode *pathListNodePtr = *pl;
    while (*pl)
    {
        *pl = (*pl)->next;
        freePath(&pathListNodePtr->path);
        pathListNodePtr = *pl;
    }
    return;
}
int pathLength(Path p)
{
    int length = 0;
    while (p)
    {
        p = p->next;
        ++length;
    }
    return length;
}
PathNode *lastPathNode(Path p)
{
    if (!p)
        ERROR("misuse of lastPathNode\n")
    else
        while (p->next)
            p = p->next;
    return p;
}
Tile relCoordinate(Direction dir, Tile t)
{
    Tile ret;

    switch (dir)
    {
        case UP:
            ret.row = t.row - 1;
            ret.col = t.col;
            break;
        case DOWN:
            ret.row = t.row + 1;
            ret.col = t.col;
            break;
        case LEFT:
            ret.row = t.row;
            ret.col = t.col - 1;
            break;
        case RIGHT:
            ret.row = t.row;
            ret.col = t.col + 1;
            break;
        default:
            ERROR("misuse of relCoordinate\n")
            break;
    }
    return ret;
}
Segment segRelFrom(Direction dir, Tile t)
{
    if (dir == UP)
        return horizSeg[t.row][t.col];
    else if (dir == DOWN)
        return horizSeg[t.row + 1][t.col];
    else if (dir == LEFT)
        return vertSeg[t.row][t.col];
    else /* dir == RIGHT */
        return vertSeg[t.row][t.col + 1];
}
Path *copyPath(Path p)
{
    Path *tmp, *ret = NULL;
    if (p)
    {
        ret = (Path *)malloc(sizeof(Path));
        if (!ret)
            ERROR("appendPathNode failed to allocate Path\n")
        else
        {
            *ret = (PathNode *)malloc(sizeof(PathNode));
            if (!*ret)
                ERROR("appendPathNode failed to allocate PathNode\n")
            else
            {
                (*ret)->tile.row = p->tile.row;
                (*ret)->tile.col = p->tile.col;
                (*ret)->tile.dir = p->tile.dir;
                (*ret)->tile.motions = p->tile.motions;
                tmp = copyPath(p->next);
                (*ret)->next = tmp ? *tmp : NULL;
            }
        }
    }
    return ret;
}
Path *appendPathNode(Path p, Tile t)
{
    Path *ret = NULL;
    ret = (Path *)malloc(sizeof(Path));
    if (!ret)
        ERROR("appendPathNode failed to allocate Path\n")
    else
    {
        *ret = (PathNode *)malloc(sizeof(PathNode));
        if (!*ret)
            ERROR("appendPathNode failed to allocate PathNode\n")
        else
        {
            if (p)
            {
                (*ret)->tile.row = p->tile.row;
                (*ret)->tile.col = p->tile.col;
                (*ret)->tile.dir = p->tile.dir;
                (*ret)->tile.motions = p->tile.motions;
                (*ret)->next = *appendPathNode(p->next, t);
            }
            else
            {
                {
                    (*ret)->tile.row = t.row;
                    (*ret)->tile.col = t.col;
                    (*ret)->tile.dir = t.dir;
                    (*ret)->tile.motions = t.motions;
                    (*ret)->next = NULL;
                }
            }
        }
    }
    return ret;
}
/* assumes p contains at least one path */
Bool expandFirstBranchInTree(PathList pl)
{
    PathListNode *pathListNodePtr = NULL;
    PathNode *lastNode = lastPathNode(pl->path); /* pointer to last node in first path in p */
    PathListNode *existingPath = NULL, *tmp = NULL;
    Tile nextNode;
    Segment nextSegment;
    Direction dir;
    int numMotions;

    if (lastNode->tile.col == 0 && lastNode->tile.row == 0) /* should only evaluate to TRUE when we start at goal */
        return TRUE;
    else
        for (dir = UP; dir < NUM_DIRECTIONS; ++dir)
        {
            nextSegment = segRelFrom(dir, lastNode->tile);
            nextNode = relCoordinate(dir, lastNode->tile); /* row, col */
            /* robot will always end up facing direction dir when it transitions to the
               next tile in relative direction dir from its current tile t */
            nextNode.dir = dir;
            nextNode.motions = lastNode->tile.motions               /* number of motions to get here so far */
                             + (abs(dir - lastNode->tile.dir) % 2)  /* number of 90 degree turns before moving */
                             + 1;                                   /* moving forward to next node */

            /* segment in direction dir from lastNode is unblocked */
            if (nextSegment == UNBLOCKED)
            {
                /* node in direction dir from lastNode is not already in a path in pathsHome */
                if(!(existingPath = pathListNodeContainingTile(nextNode, pathsHome, &numMotions)))
                {
                    /* (1) allocate space for a new list */
                    pathListNodePtr = (PathListNode *)malloc(sizeof(PathListNode));
                    if (!pathListNodePtr)
                        ERROR("Failed to allocate memory for pathListNodePtr in oneDirection\n")
                    else
                    {
                        /* (2) create a new list: same as first path in p, except with a new node
                               in direction dir from last node added to the end */
                        pathListNodePtr->path = *appendPathNode(pl->path, nextNode);

                        /* (3) add the new list to the front of pathsHome */
                        pathListNodePtr->next = pathsHome;
                        pathsHome = pathListNodePtr;
                    }
                }
                /* pathsHome does contain the node, but the number of turns taken to get there is smaller
                   in the new path */
                else if (nextNode.motions < numMotions)
                {
                    /* (0) remove path containing existing node from pathsHome */
                    tmp = existingPath->next;
                    freePath(&existingPath->path);
                    existingPath = tmp;

                    /* (1) allocate space for a new list */
                    pathListNodePtr = (PathListNode *)malloc(sizeof(PathListNode));
                    if (!pathListNodePtr)
                        ERROR("Failed to allocate memory for pathListNodePtr in oneDirection\n")
                    else
                    {
                        /* (2) create a new list: same as first path in p, except with a new node
                               in direction dir from last node added to the end */
                        pathListNodePtr->path = *appendPathNode(pl->path, nextNode);

                        /* (3) add the new list to the front of pathsHome */
                        pathListNodePtr->next = pathsHome;
                        pathsHome = pathListNodePtr;
                    }
                }

                /* if we have found a clear path home, we return TRUE */
                if (nextNode.row == 0 && nextNode.col == 0)
                    return TRUE;
            }
        }
    /* we did not find a clear path home, so we return FALSE */
    return FALSE;
}
Bool expandTreeOneLevel(PathList pl)
{
    Bool foundPathHome;
    PathList ptr = pl;
    while (ptr)
    {
        foundPathHome = expandFirstBranchInTree(ptr);
        if (foundPathHome)
            return TRUE;
        else
            ptr = ptr->next;
    }
    /* did not find a path home in this tree level */
    return FALSE;
}
/* In LISP-speak:
 * (define pruneTree
 *   (lambda (pl)
 *     (cond
 *       ((null? pl)
 *        (quote ()))
 *       ((< (length (car pl)) pathSize)
 *        (pruneTree (cdr pl)))
 *       (else
 *        (cons (car pl) (pruneTree (cdr pl)))))))
 */
PathList *pruneTree(PathList *pl)
{
    PathList *ptr;
    if (!pl || !*pl)
        return NULL;
    else if (pathLength((*pl)->path) < pathSize)
    {
        ptr = pl;
        pl = &(*pl)->next;
        freePath(&(*ptr)->path);
        return pruneTree(pl);
    }
    else
    {
        ptr = pruneTree(&(*pl)->next);
        (*pl)->next = ptr ? *ptr : NULL;
        return pl;
    }
}
Path *branchAndBound(void)              /* assumes there is always at least one path home */
{
    PathList *tmp = NULL;
    Bool stuck = FALSE;
    while (!stuck)
    {
        if (expandTreeOneLevel(pathsHome))
            return copyPath(pathsHome->path);    /* first path in pathsHome */
        else
        {
            ++pathSize;
            tmp =  pruneTree(&pathsHome);
            if (tmp)
                pathsHome = *tmp;
            else
                stuck = TRUE;
        }
    }
    /* return NULL if there is no possible way to get home */
    return NULL;
}
/* needed as function pointer argument in wallHug */
Bool inLeftMostColumnOrBottomRow(void *dummy)
{
    return current[COL] == 0 || current[ROW] == NUM_ROWS - 1;
}
/* needed as function pointer argument in wallHug */
Bool inRightMostColumnOrTopRow(void *dummy)
{
    return current[COL] == NUM_COLS - 1 || current[ROW] == 0;
}
void returnHome(void)
{
    Tile first;
    PathListNode *tmp = (PathListNode *)malloc(sizeof(PathListNode));

    do
    {
        /*  add current node to pathsHome */
        first.row = current[ROW];
        first.col = current[COL];
        first.dir = direction;
        first.motions = 0;
        tmp->path = *appendPathNode(NULL, first);
        tmp->next = pathsHome;
        pathsHome = tmp;

        /* initialize pathSize */
        pathSize = 1;

        /* descend into the dark, murky depths of constructing, navigating, and deallocating a quad tree */
        fastestPath = branchAndBound();
        freePathList(&pathsHome);

        /* if there is no way to get home */
        if (!fastestPath)
        {
            turnAbs(UP);
            while (segAbs(UP) != BLOCKED)
            {
                moveForward();
            }
            /* TODO: Break out of this when you can calculate */
            while (!(current[ROW] == 0 && current[COL] == 0))
            {
                turnAbs(LEFT);
                wallHugUntil(RIGHT, inLeftMostColumnOrBottomRow, NULL);
                turnAbs(RIGHT);
                wallHugUntil(LEFT, inRightMostColumnOrTopRow, NULL);
            }
            break;
        }

    } while (!followPath(*fastestPath ? (*fastestPath)->next : NULL));

    return;
}
/* TEST ONLY */
#ifdef VIRTUAL_BOT

char *DirectionToStr(Direction dir)
{
    switch (dir)
    {
        case UP:    return "UP";        break;
        case DOWN:  return "DOWN";      break;
        case LEFT:  return "LEFT";      break;
        case RIGHT: return "RIGHT";     break;
        default:    return "INVALID";   break;
    }
}

void displayPath(Path p)
{
    Path ptr = p; /* <<< SHOULDN'T NEED ptr BUT JUST WANTED TO MAKE SURE p WASN'T ALTERED */
    printf("START");
    while (ptr)
    {
        printf("(%d, %d)", ptr->tile.row, ptr->tile.col);
        ptr = ptr->next;
    }
    printf("END\n");

    return;
}
void displayPathList(PathList pl)
{
    int i = 0;
    PathList ptr = pl;
    while (ptr)
    {
        printf("%d: ", i++);
        displayPath(ptr->path);
        ptr = ptr->next;
    }
    printf("\n");
    return;
}
int getTileIndex(int a, int b)
{
    int i = 0;
    Path ptr = *fastestPath;
    while (ptr)
    {
        if (ptr->tile.row == a && ptr->tile.col == b)
            return i;
        else
        {
            ptr = ptr->next;
            ++i;
        }
    }
    /* (a, b) not in fastestPath */
    return -1;
}
#endif
/* followPath takes a linked list of 2-dimensional arrays representing nodes on the grid and attempts to follow that path.
 * If robot successfully makes it to end of path, TRUE is returned; otherwise, FALSE (return with error)
 */
Bool followPath(Path path)
{
    Direction dir;

    while (path)
    {
        /* next node is UP */
        if ((current[ROW] == path->tile.row + 1) && (current[COL] == path->tile.col))
            dir = UP;
        /* next node is DOWN */
        else if ((current[ROW] == path->tile.row - 1) && (current[COL] == path->tile.col))
            dir = DOWN;
        /* next node is LEFT */
        else if ((current[ROW] == path->tile.row) && (current[COL] == path->tile.col + 1))
            dir = LEFT;
        /* next node is RIGHT */
        else if ((current[ROW] == path->tile.row) && (current[COL] == path->tile.col - 1))
            dir = RIGHT;

        /* TURN TO FACE APPROPRIATE DIRECTION */
        turnAbs(dir);

        /* move to next node and scan for surrounding blocks if able, otherwise return with error */
        if (segRel(FRONT) == BLOCKED)
            return FALSE;
        else
        {
            moveForward();
            if (segmentsChanged)
                return FALSE;
            else
                path = path->next;
        }
    }

    return TRUE;
}
/*}}}*/
/*{{{ VIRTUAL OBJECT PRESENCE FUNCTIONS */
#ifdef VIRTUAL_BOT
/* returns status of adjacent segment, specified by absolute Direction (think of UP, RIGHT, DOWN, and LEFT and North,
   THIS FUNCTION IS USED TO INDICATE THE PRESENCE OF A VIRTUAL GRID BLOCK */
Segment blockedSegAbs(Direction absDir)
{
    if (absDir == UP)
        return blockedHorizSeg[current[ROW]][current[COL]];
    else if (absDir == DOWN)
        return blockedHorizSeg[current[ROW] + 1][current[COL]];
    else if (absDir == LEFT)
        return blockedVertSeg[current[ROW]][current[COL]];
    else /* absDir == RIGHT */
        return blockedVertSeg[current[ROW]][current[COL] + 1];
}

/* returns status of adjacent segment, specified by relative Direction
   THIS FUNCTION IS USED TO INDICATE THE PRESENCE OF A VIRTUAL GRID BLOCK */
Segment blockedSegRel(Direction relSeg)
{
    Segment returnValue;

    /* currently facing absolute UP */
    if (direction == UP)
    {
        returnValue = blockedSegAbs(relSeg); /* same as blockedSegAbs function */
    }
    /* currently facing absolute DOWN */
    else if (direction == DOWN)
    {
        if (relSeg == FRONT) /* absolute DOWN */
            returnValue = blockedSegAbs(DOWN);
        else if (relSeg == BACK) /* absolute UP */
            returnValue = blockedSegAbs(UP);
        else if (relSeg == LEFT) /* absolute RIGHT */
            returnValue = blockedSegAbs(RIGHT);
        else /* relSeg == RIGHT ---> absolute LEFT */
            returnValue = blockedSegAbs(LEFT);
    }
    /* currently facing absolute LEFT */
    else if (direction == LEFT)
    {
        if (relSeg == FRONT) /* absolute LEFT */
            returnValue = blockedSegAbs(LEFT);
        else if (relSeg == BACK) /* absolute RIGHT */
            returnValue = blockedSegAbs(RIGHT);
        else if (relSeg == LEFT) /* absolute DOWN */
            returnValue = blockedSegAbs(DOWN);
        else /* relSeg == RIGHT ---> absolute UP */
            returnValue = blockedSegAbs(UP);
    }
    /* currently facing absolute RIGHT */
    else /* direction == RIGHT */
    {
        if (relSeg == FRONT) /* absolute RIGHT */
            returnValue = blockedSegAbs(RIGHT);
        else if (relSeg == BACK) /* absolute LEFT */
            returnValue = blockedSegAbs(LEFT);
        else if (relSeg == LEFT) /* absolute UP */
            returnValue = blockedSegAbs(UP);
        else /* relSeg == RIGHT ---> absolute DOWN */
            returnValue = blockedSegAbs(DOWN);
    }

    return returnValue;
}

#ifdef LONG_RANGE_SENSORS
/* returns status of adjacent segment, specified by absolute Direction (think of UP, RIGHT, DOWN, and LEFT and North,
   THIS FUNCTION IS USED TO INDICATE THE PRESENCE OF A VIRTUAL GRID BLOCK */
Segment blockedSegAbs2(Direction absDir)
{
    if (absDir == UP)
        return blockedHorizSeg[current[ROW] - 1][current[COL]];
    else if (absDir == DOWN)
        return blockedHorizSeg[current[ROW] + 2][current[COL]];
    else if (absDir == LEFT)
        return blockedVertSeg[current[ROW]][current[COL] - 1];
    else /* absDir == RIGHT */
        return blockedVertSeg[current[ROW]][current[COL] + 2];
}

/* returns status of adjacent segment, specified by relative Direction
   THIS FUNCTION IS USED TO INDICATE THE PRESENCE OF A VIRTUAL GRID BLOCK */
Segment blockedSegRel2(Direction relSeg)
{
    Segment returnValue;

    /* currently facing absolute UP */
    if (direction == UP)
    {
        returnValue = blockedSegAbs2(relSeg); /* same as blockedSegAbs2 function */
    }
    /* currently facing absolute DOWN */
    else if (direction == DOWN)
    {
        if (relSeg == FRONT) /* absolute DOWN */
            returnValue = blockedSegAbs2(DOWN);
        else if (relSeg == BACK) /* absolute UP */
            returnValue = blockedSegAbs2(UP);
        else if (relSeg == LEFT) /* absolute RIGHT */
            returnValue = blockedSegAbs2(RIGHT);
        else /* relSeg == RIGHT ---> absolute LEFT */
            returnValue = blockedSegAbs2(LEFT);
    }
    /* currently facing absolute LEFT */
    else if (direction == LEFT)
    {
        if (relSeg == FRONT) /* absolute LEFT */
            returnValue = blockedSegAbs2(LEFT);
        else if (relSeg == BACK) /* absolute RIGHT */
            returnValue = blockedSegAbs2(RIGHT);
        else if (relSeg == LEFT) /* absolute DOWN */
            returnValue = blockedSegAbs2(DOWN);
        else /* relSeg == RIGHT ---> absolute UP */
            returnValue = blockedSegAbs2(UP);
    }
    /* currently facing absolute RIGHT */
    else /* direction == RIGHT */
    {
        if (relSeg == FRONT) /* absolute RIGHT */
            returnValue = blockedSegAbs2(RIGHT);
        else if (relSeg == BACK) /* absolute LEFT */
            returnValue = blockedSegAbs2(LEFT);
        else if (relSeg == LEFT) /* absolute UP */
            returnValue = blockedSegAbs2(UP);
        else /* relSeg == RIGHT ---> absolute DOWN */
            returnValue = blockedSegAbs2(DOWN);
    }

    return returnValue;
}
#endif

/* returns TRUE or FALSE, depending on whether or not the enemy robot is occupying the segment in the absolute direction
   away from the current segment (1 node away) */
Bool enemyPresenceAbs(Direction absDir)
{
    if (absDir == UP && current[ROW] > 0)
        return enemy[ROW] == current[ROW] - 1 && enemy[COL] == current[COL] ? TRUE : FALSE;
    else if (absDir == DOWN && current[ROW] < NUM_ROWS - 1)
        return enemy[ROW] == current[ROW] + 1 && enemy[COL] == current[COL] ? TRUE : FALSE;
    else if (absDir == LEFT && current[COL] > 0)
        return enemy[ROW] == current[ROW] && enemy[COL] == current[COL] - 1 ? TRUE : FALSE;
    else if (absDir == RIGHT && current[COL] < NUM_COLS - 1)
        return enemy[ROW] == current[ROW] && enemy[COL] == current[COL] + 1 ? TRUE : FALSE;
    else
        return FALSE;
}

/* same as enemyPresenceAbs, except direction is specified in relative terms instead of absolute */
Bool enemyPresenceRel(Direction relSeg)
{
    Bool returnValue;

    /* currently facing absolute UP */
    if (direction == UP)
    {
        returnValue = enemyPresenceAbs(relSeg); /* same as enemyPresenceAbs function */
    }
    /* currently facing absolute DOWN */
    else if (direction == DOWN)
    {
        if (relSeg == FRONT) /* absolute DOWN */
            returnValue = enemyPresenceAbs(DOWN);
        else if (relSeg == BACK) /* absolute UP */
            returnValue = enemyPresenceAbs(UP);
        else if (relSeg == LEFT) /* absolute RIGHT */
            returnValue = enemyPresenceAbs(RIGHT);
        else /* relSeg == RIGHT ---> absolute LEFT */
            returnValue = enemyPresenceAbs(LEFT);
    }
    /* currently facing absolute LEFT */
    else if (direction == LEFT)
    {
        if (relSeg == FRONT) /* absolute LEFT */
            returnValue = enemyPresenceAbs(LEFT);
        else if (relSeg == BACK) /* absolute RIGHT */
            returnValue = enemyPresenceAbs(RIGHT);
        else if (relSeg == LEFT) /* absolute DOWN */
            returnValue = enemyPresenceAbs(DOWN);
        else /* relSeg == RIGHT ---> absolute UP */
            returnValue = enemyPresenceAbs(UP);
    }
    /* currently facing absolute RIGHT */
    else /* direction == RIGHT */
    {
        if (relSeg == FRONT) /* absolute RIGHT */
            returnValue = enemyPresenceAbs(RIGHT);
        else if (relSeg == BACK) /* absolute LEFT */
            returnValue = enemyPresenceAbs(LEFT);
        else if (relSeg == LEFT) /* absolute UP */
            returnValue = enemyPresenceAbs(UP);
        else /* relSeg == RIGHT ---> absolute DOWN */
            returnValue = enemyPresenceAbs(DOWN);
    }

    return returnValue;
}

#ifdef LONG_RANGE_SENSORS
/* returns TRUE or FALSE, depending on whether or not the enemy robot is occupying the segment in the absolute direction
   away from the current segment (2 nodes away) */
Bool enemyPresenceAbs2(Direction absDir)
{
    if (absDir == UP && current[ROW] > 1)
        return enemy[ROW] == current[ROW] - 2 && enemy[COL] == current[COL] ? TRUE : FALSE;
    else if (absDir == DOWN && current[ROW] < NUM_ROWS - 2)
        return enemy[ROW] == current[ROW] + 2 && enemy[COL] == current[COL] ? TRUE : FALSE;
    else if (absDir == LEFT && current[COL] > 1)
        return enemy[ROW] == current[ROW] && enemy[COL] == current[COL] - 2 ? TRUE : FALSE;
    else if (absDir == RIGHT && current[COL] < NUM_COLS - 2)
        return enemy[ROW] == current[ROW] && enemy[COL] == current[COL] + 2 ? TRUE : FALSE;
    else
        return FALSE;
}

/* same as enemyPresenceAbs2, except direction is specified in relative terms instead of absolute */
Bool enemyPresenceRel2(Direction relSeg)
{
    Bool returnValue;

    /* currently facing absolute UP */
    if (direction == UP)
    {
        returnValue = enemyPresenceAbs2(relSeg); /* same as enemyPresenceAbs2 function */
    }
    /* currently facing absolute DOWN */
    else if (direction == DOWN)
    {
        if (relSeg == FRONT) /* absolute DOWN */
            returnValue = enemyPresenceAbs2(DOWN);
        else if (relSeg == BACK) /* absolute UP */
            returnValue = enemyPresenceAbs2(UP);
        else if (relSeg == LEFT) /* absolute RIGHT */
            returnValue = enemyPresenceAbs2(RIGHT);
        else /* relSeg == RIGHT ---> absolute LEFT */
            returnValue = enemyPresenceAbs2(LEFT);
    }
    /* currently facing absolute LEFT */
    else if (direction == LEFT)
    {
        if (relSeg == FRONT) /* absolute LEFT */
            returnValue = enemyPresenceAbs2(LEFT);
        else if (relSeg == BACK) /* absolute RIGHT */
            returnValue = enemyPresenceAbs2(RIGHT);
        else if (relSeg == LEFT) /* absolute DOWN */
            returnValue = enemyPresenceAbs2(DOWN);
        else /* relSeg == RIGHT ---> absolute UP */
            returnValue = enemyPresenceAbs2(UP);
    }
    /* currently facing absolute RIGHT */
    else /* direction == RIGHT */
    {
        if (relSeg == FRONT) /* absolute RIGHT */
            returnValue = enemyPresenceAbs2(RIGHT);
        else if (relSeg == BACK) /* absolute LEFT */
            returnValue = enemyPresenceAbs2(LEFT);
        else if (relSeg == LEFT) /* absolute UP */
            returnValue = enemyPresenceAbs2(UP);
        else /* relSeg == RIGHT ---> absolute DOWN */
            returnValue = enemyPresenceAbs2(DOWN);
    }

    return returnValue;
}
#endif

#endif
/* }}} */
/*{{{ CONSOLE OUTPUT TEST FUNCTIONS */
#ifdef VIRTUAL_BOT
/* FUNCTION displayGrid
   0   1   2   3   4   5
 +###+###+###+###+###+###+
0#   |   |   |   |   |   #
 +---+---+---+---+---+---+
1#   |   |   |   |   |   #
 +---+---+---+---+---+---+
2#   |   |   |   |   |   #
 +---+---+---+---+---+---+
3#   |   |   |   |   |   #
 +---+---+---+---+---+---+
4#   |   |   |   |   |   #
 +---+---+---+---+---+---+
5#   |   |   |   |   |   #
 +---+---+---+---+---+---+
6#   |   |   |   |   |   #
 +###+###+###+###+###+###+

 Direction indicators:
   ^
 <   >
   v

  Block indicators:
  ? / ???   unsensed    unblocked
  | / ---   sensed      unblocked
  B / BBB   unsensed    blocked
  # / ###   sensed      blocked
  % / %%%   sensed      enemy robot blocked
  NOTE: Robot cannot tell the difference between # and %

  Other indicators:
  *     Enemy Robot
  F     Flag
  .     VISITED node
*/
void displayGrid(void)
{
    int i, j;           /* LCV's */
    int idx;            /* index of return home tile */
    Bool displayCursor; /* whether or not to display cursor on segment */

    CLEAR
    if (setupMode)
        if (cursorMode == HORIZ)
            printf("blockedHorizSeg: %d, %d\n", cursor[ROW], cursor[COL]);
        else
            printf("blockedVertSeg: %d, %d\n", cursor[ROW], cursor[COL]);
    else
    {
        printf("current: %d, %d\t", current[ROW], current[COL]);
        printf("enemy: %d, %d\n", enemy[ROW], enemy[COL]);
    }

    /* display header */
    printf("%*s", rowNumWidth, "");
    for (i = 0; i < NUM_COLS; ++i)
        printf("%4d", i);
    printf("\n%*s ", rowNumWidth, "");

    /* display bulk of grid */
    for (i = 0; i < NUM_ROWS; ++i)
    {
        /* horizontal segments */
        for (j = 0; j < NUM_COLS; ++j)
        {
            displayCursor = setupMode && cursorMode == HORIZ && cursor[ROW] == i && cursor[COL] == j;
            printf("+");

            if (horizSeg[i][j] == IDK)
            {
                if (blockedHorizSeg[i][j] == BLOCKED)
                    if (displayCursor)
                        printf("[B]");
                    else
                        printf("BBB");
                else
                    if (displayCursor)
                        printf("[?]");
                    else
                        printf("???");
            }
            else
            {
                if (horizSeg[i][j] == UNBLOCKED) {
                    if (displayCursor)
                        printf("[-]");
                    else
                        printf("---");
                }
                else /* horizSeg[i][j] == BLOCKED */ {
                    if (blockedHorizSeg[i][j] == BLOCKED || i == 0 || i == NUM_HORIZ_SEG_ROWS - 1)
                        if (displayCursor)
                            printf("[#]");      /* actual block */
                        else
                            printf("###");      /* actual block */
                    else
                    {
                        if (displayCursor)
                            printf("[%%]");     /* sensed enemy robot */
                        else
                            printf("%%%%%%");   /* sensed enemy robot */
                    }
                }
            }
        }

        /* new line and row number */
        printf("+\n%*d ", rowNumWidth, i);

        /* vertical segments and node information */
        for (j = 0; j < NUM_COLS; ++j)
        {
            displayCursor = setupMode && cursorMode == VERT && cursor[ROW] == i && cursor[COL] == j;

            /* segments */
            if (vertSeg[i][j] == IDK)
            {
                if (blockedVertSeg[i][j] == BLOCKED)
                    if (displayCursor)
                        printf("\b[B]");
                    else
                        printf("B ");
                else
                    if (displayCursor)
                        printf("\b[?]");
                    else
                        printf("? ");
            }
            else
            {
                if (vertSeg[i][j] == UNBLOCKED)
                    if (displayCursor)
                        printf("\b[|]");
                    else
                        printf("| ");
                else /* horizSeg[i][j] == BLOCKED */
                    if (blockedVertSeg[i][j] == BLOCKED || j == 0 || j == NUM_COLS)
                        if (displayCursor)
                            printf("\b[#]");    /* actual block or boundary */
                        else
                            printf("# ");       /* actual block or boundary */
                    else
                        if (displayCursor)
                            printf("\b[%%]");   /* sensed enemy robot */
                        else
                            printf("%% ");      /* sensed enemy robot */
            }

            /* node information */
            if (i == current[ROW] && j == current[COL])     /* display robot */
            {
                if (direction == UP)
                    printf("^ ");
                else if (direction == DOWN)
                    printf("v ");
                else if (direction == LEFT)
                    printf("< ");
                else /* direction == RIGHT */
                    printf("> ");
            }
            else if (i == enemy[ROW] && j == enemy[COL])    /* display enemy robot */
            {
                printf("* ");
            }
            else if (haveFlag == FALSE && i == NUM_ROWS - 1 && j == NUM_COLS - 1)
            {
                printf("F ");
            }
            else if (fastestPath && (idx = getTileIndex(i, j)) >= 0)
            {
                printf("\b(%d)", idx % 10);
            }
            #ifdef REMEMBER_VISITED_NODES
            else if (grid[i][j] == VISITED) /* display VISITED indicator */
            {
                printf(". ");
            }
            #endif
            else /* display blank -- UNVISITED indicator */
            {
                printf("  ");
            }
        } /* for */

        /* last column of vertical segments */
        if (vertSeg[i][NUM_VERT_SEG_COLS - 1] == IDK) /* this should never evaluate to TRUE */
        {
            if (blockedVertSeg[i][NUM_VERT_SEG_COLS - 1] == BLOCKED)
                printf("B\n  ");
            else
                printf("?\n  ");
        }
        else
        {
            if (vertSeg[i][NUM_VERT_SEG_COLS] == UNBLOCKED)
                printf("|\n%*s ", rowNumWidth, "");
            else /* BLOCKED */
                printf("#\n%*s ", rowNumWidth, "");
        }
    }

    /* last row of horizontal segments */
    for (j = 0; j < NUM_COLS; ++j)
    {
        printf("+");

        if (horizSeg[NUM_HORIZ_SEG_ROWS - 1][j] == IDK) /* this should never evaluate to TRUE */
        {
            if (blockedHorizSeg[NUM_HORIZ_SEG_ROWS - 1][j] == BLOCKED)
                printf("BBB");
            else
                printf("???");
        }
        else
        {
            if (horizSeg[NUM_HORIZ_SEG_ROWS - 1][j] == UNBLOCKED)
                printf("---");
            else /* BLOCKED */
                printf("###");
        }
    }
    printf("+\n  ");

    if (setupMode)
        printf("Setup Mode\n");
    else
        printf("Test Mode\n");

    if (haveFlag)
        printf("CAPTURED THE FLAG!\n");

    return;
}

/* take interactive control of the virtual enemy robot */
void controlEnemyRobot(void)
{
    do {
        displayGrid();
        buffer = getchar();
        if      (ctrlUp(buffer))     moveEnemyRobot(UP);
        else if (ctrlDown(buffer))   moveEnemyRobot(DOWN);
        else if (ctrlLeft(buffer))   moveEnemyRobot(LEFT);
        else if (ctrlRight(buffer))  moveEnemyRobot(RIGHT);
        else if (ctrlDone(buffer))   { printf("\b*** Stopped ***\n"); exit(0); }
    } while(!advanceRobot(buffer));

    return;
}

/* move enemy robot on grid
 * NOTES:
 * - These if statements are very fragile -- do not alter the scoping rules or order!
 * - The function is designed to allow the enemy robot to skip over its obstacles, including your robot
 */
void moveEnemyRobot(Direction dirToMove)
{
    switch (dirToMove)
    {
    case UP:    if (current[ROW] == enemy[ROW] - 1 && current[COL] == enemy[COL])
                {
                    if (enemy[ROW] > 1)
                        enemy[ROW] -= 2;
                }
                else if (enemy[ROW] > 0)
                    enemy[ROW] -= 1;
                break;
    case DOWN:  if (current[ROW] == enemy[ROW] + 1 && current[COL] == enemy[COL])
                {
                    if (enemy[ROW] < NUM_ROWS - 2)
                        enemy[ROW] += 2;
                }
                else if (enemy[ROW] < NUM_ROWS - 1)
                    enemy[ROW] += 1;
                break;
    case LEFT:  if (current[ROW] == enemy[ROW] && current[COL] == enemy[COL] - 1)
                {
                    if (enemy[COL] > 1)
                        enemy[COL] -= 2;
                }
                else if (enemy[COL] > 0)
                    enemy[COL] -= 1;
                break;
    case RIGHT: if (current[ROW] == enemy[ROW] && current[COL] == enemy[COL] + 1)
                {
                    if (enemy[COL] < NUM_COLS - 2)
                        enemy[COL] += 2;
                }
                else if (enemy[COL] < NUM_COLS - 1)
                    enemy[COL] += 1;
                break;
    default:
                ERROR("Received an invalid direction in function moveEnemyRobot\n")
                break;
    };

    return;
}

/* move cursor in the specified direction, if possible */
void moveCursor(Direction dirToMove)
{
    switch (dirToMove)
    {
    case UP:    if ((cursorMode == HORIZ && cursor[ROW] > 1) ||
                    (cursorMode == VERT  && cursor[ROW] > 0))
                    --cursor[ROW];
                break;
    case DOWN:  if (cursor[ROW] < NUM_ROWS - 1)
                    ++cursor[ROW];
                break;
    case LEFT:  if ((cursorMode == HORIZ && cursor[COL] > 0) ||
                    (cursorMode == VERT  && cursor[COL] > 1))
                    --cursor[COL];
                break;
    case RIGHT: if (cursor[COL] < NUM_COLS - 1)
                    ++cursor[COL];
                break;
    default:
                ERROR("Received an invalid direction in function moveCursor\n")
                break;
    };

    return;
}

/* toggle presence/absence of block at cursor position -- used in initializeTestVariables */
void toggleBlock(void)
{
    if (cursorMode == HORIZ)
        if (blockedHorizSeg[cursor[ROW]][cursor[COL]] == UNBLOCKED)
            blockedHorizSeg[cursor[ROW]][cursor[COL]] = BLOCKED;
        else
            blockedHorizSeg[cursor[ROW]][cursor[COL]] = UNBLOCKED;
    else /* cursorMode == VERT */
        if (blockedVertSeg[cursor[ROW]][cursor[COL]] == UNBLOCKED)
            blockedVertSeg[cursor[ROW]][cursor[COL]] = BLOCKED;
        else
            blockedVertSeg[cursor[ROW]][cursor[COL]] = UNBLOCKED;

    return;
}

/* toggle cursorMode and reposition cursor on grid -- used in initializeTestVariables */
void toggleVertHoriz(void)
{
    if (cursorMode == HORIZ)
    {
        if (cursor[COL] > 0)
        {
            cursorMode = VERT;
        }
    }
    else /* cursorMode == VERT */
    {
        if (cursor[ROW] < NUM_ROWS - 1)
        {
            cursorMode = HORIZ;
            ++cursor[ROW];
            --cursor[COL];
        }
    }

    /* TODO: Read test case from file */
    /* FILE *fp; */
    /* fp = fopen("1.test", "r"); */
    /* int i, j; */
    /* fscanf(fp, "\n"); */
    /* for (i = 0; i < NUM_HORIZ_SEG_ROWS; ++i) */
    /*     for (j = 0; j < NUM_VERT_SEG_COLS; ++j) */
    /*         fscanf(fp, "%d", blockedHorizSeg[i][j]); */
    /* fscanf(fp, "\n"); */
    /* for (i = 0; i < NUM_ROWS; ++i) */
    /*     for (j = 0; j < NUM_COLS; ++j) */
    /*         fscanf(fp, "%d", blockedVertSeg[i][j]); */

    return;
}

/* prime the terminal for interactive control */
void initializeTestControls(void)
{
#ifdef _WIN32
    /* TODO: Add Windows support for interactive console text */
#else
    /* change terminal settings so there is no need to press ENTER on input */
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
#endif
    return;
}

/* restore old terminal I/O settings */
void tearDownTestControls(void)
{
#ifdef _WIN32
    /* TODO: Add Windows support for interactive console text */
#else
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
#endif
    return;
}

/* Waits for the user to press ENTER */
void waitForEnter(void)
{
    char enter;
    enter = 0;

    while (enter != '\n' && enter != EOF)
        enter = getchar();

    return;
}

/* Waits for the user to press ENTER, and then clears the screen. */
void pauseAndClear(void)
{
    waitForEnter();
    CLEAR
    return;
}

/* getline:  read a line, return length */
int getline(char *line, int max)
{
    printf("In getline...\n"); /* TEST */
    /* restore terminal to old settings */
    tearDownTestControls();

    if (!fgets(line, max, stdin))
        return 0;
    else {
        line[strlen(line) - 1] = '\0';
        return strlen(line);
    }

    /* go back to test controls */
    initializeTestControls();
    printf("Leaving getline...\n"); /* TEST */
    pauseAndClear(); /* TEST */
}

#elif defined(DEBUG_GRID)

/* FUNCTION:     displayDebugGrid
   DESCRIPTION:  Displays the robot's memory of the physical grid and spacial parameters on a serial monitor while
                 connected to microcontroller.

   0   1   2   3   4   5
 +###+###+###+###+###+###+
0#   |   |   |   |   |   #
 +---+---+---+---+---+---+
1#   |   |   |   |   |   #
 +---+---+---+---+---+---+
2#   |   |   |   |   |   #
 +---+---+---+---+---+---+
3#   |   |   |   |   |   #
 +---+---+---+---+---+---+
4#   |   |   |   |   |   #
 +---+---+---+---+---+---+
5#   |   |   |   |   |   #
 +---+---+---+---+---+---+
6#   |   |   |   |   |   #
 +###+###+###+###+###+###+

 Direction indicators:
   ^
 <   >
   v

  Block indicators:
  ? / ???   unsensed    could be either blocked or unblocked (IDK)
  | / ---   sensed      unblocked
  # / ###   sensed      blocked

  Other indicators:
  F     Flag
  .     VISITED node
*/
void displayDebugGrid(void)
{
    int i, j;           /* LCV's */

    CLEAR
    printf("current: %d, %d\n", current[ROW], current[COL]);

    /* display header */
    printf("    0   1   2   3   4   5\n  ");

    /* display bulk of grid */
    for (i = 0; i < NUM_ROWS; ++i)
    {
        /* horizontal segments */
        for (j = 0; j < NUM_COLS; ++j)
        {
            printf("+");
            switch (horizSeg[i][j])
            {
                case IDK:       printf("???"); break;
                case BLOCKED:   printf("###"); break;
                case UNBLOCKED: printf("---"); break;
                /* no default -- we make no errors! */
            }
        }
        printf("+\n%d ", i); /* new line and row number */

        /* vertical segments and node information */
        for (j = 0; j < NUM_COLS; ++j)
        {
            /* segments */
            switch (vertSeg[i][j])
            {
                case IDK:       printf("? "); break;
                case BLOCKED:   printf("# "); break;
                case UNBLOCKED: printf("| "); break;
                /* no default -- we make no errors! */
            }

            /* node information */
            if (i == current[ROW] && j == current[COL])     /* display robot */
            {
                if (direction == UP)
                    printf("^ ");
                else if (direction == DOWN)
                    printf("v ");
                else if (direction == LEFT)
                    printf("< ");
                else /* direction == RIGHT */
                    printf("> ");
            }
            else if (haveFlag == FALSE && i == NUM_ROWS - 1 && j == NUM_COLS - 1)
            {
                printf("F ");
            }
            #ifdef REMEMBER_VISITED_NODES
            else if (grid[i][j] == VISITED) /* display VISITED indicator */
            {
                printf(". ");
            }
            #endif
            else /* display blank -- UNVISITED indicator */
            {
                printf("  ");
            }
        } /* for */

        /* last column of vertical segments */
        switch (vertSeg[i][NUM_COLS])
        {
            case IDK:       printf("? \n  "); break; /* this should never evaluate to TRUE */
            case BLOCKED:   printf("# \n  "); break;
            case UNBLOCKED: printf("| \n  "); break;
            /* no default -- we make no errors! */
        }
    }

    /* last row of horizontal segments */
    for (j = 0; j < NUM_COLS; ++j)
    {
        printf("+");
        switch (horizSeg[NUM_ROWS][j])
        {
            case IDK:       printf("???"); break; /* this should never evaluate to TRUE */
            case BLOCKED:   printf("###"); break;
            case UNBLOCKED: printf("---"); break;
            /* no default -- we make no errors! */
        }
    }
    printf("+\n  Test Mode\n");

    if (haveFlag)
        printf("CAPTURED THE FLAG!\n");

    return;
}
#endif
/* }}} */
