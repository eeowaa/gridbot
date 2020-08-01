#lang scheme
; NOTE:
; To run this program:
; 1) Download the Dr. Racket IDE: http://racket-lang.org/download/
; 2) Open this file in Dr. Racket
; 3) Click "Run" or press Ctrl+R
; 4) Follow instructions under the "TEST CONDITIONS" header near the
;    bottom of this file to run specific test cases
; 5) Add tests as desired (the pattern will be easy to pick up, even
;    if you don't know Scheme or another dialect of LISP)
;
; PROGRAM DESCRIPTION:
; Scheme approach to solving return-home problem for robotics club.
; Note that this program only accounts for a small subset of the overall
; grid-traversal problem. (It only accounts for figuring out a branch-and-
; bound search to apply back to the original C-code.)

; UTILITY FUNCTIONS
(define length
  (lambda (l)
    (cond
      ((null? l) 0)
      (else (+ 1 (length (cdr l)))))))

(define contains?
  (lambda (s l)
    (cond
      ((null? l) #f)
      ((eq? s (car l)) #t)
      (else (contains? s (cdr l))))))

(define last
  (lambda (l)
    (cond
      ((null? l) #f) ; in the case of (last '())
      ((null? (cdr l)) (car l))
      (else (last (cdr l))))))

(define append
  (lambda (s l)
    (cond
      ((null? l) (list s))
      (else (cons (car l) (append s (cdr l)))))))

(define after
  (lambda (s l)
    (cond
      ((null? l) #f) ; l does not contain s
      ((eq? s (car l))
       (cond
         ((null? (cdr l)) #f) ; s is the last expression
         (else (car (cdr l)))))
      (else (after s (cdr l))))))

; GRID CONSTANTS
(define num-grid-rows 7)
(define num-grid-cols 6)
(define visited #\y)
(define unvisited #\n)

; SEGMENT CONSTANTS
(define num-horiz-seg-rows (+ 1 num-grid-rows))
(define num-horiz-seg-cols num-grid-cols)
(define num-vert-seg-rows num-grid-rows)
(define num-vert-seg-cols (+ 1 num-grid-cols))
(define blocked #\b)
(define clear #\c)
(define unknown #\?)

; DIRECTIONS
(define up #\u)
(define right #\r)
(define down #\d)
(define left #\l)
(define directions (list up right down left))

; CURRENT POSITION IN GRID
(define current '())
(set! current '(0 0))

; COORDINATE SYSTEMS
(define make-coordinate-system
  (lambda (num-rows num-cols)
    (let make ((row 0) (col 0))
      (cond
        ((= col num-cols)
         (cond
           ((= row (- num-rows 1))
            (quote ()))
           (else
            (make (+ 1 row) 0))))
        (else
         (cons (list row col) (make row (+ 1 col))))))))

(define grid-coordinates (make-coordinate-system
                          num-grid-rows
                          num-grid-cols))
(define horiz-seg-coordinates (make-coordinate-system
                               num-horiz-seg-rows
                               num-horiz-seg-cols))
(define vert-seg-coordinates (make-coordinate-system
                              num-vert-seg-rows
                              num-vert-seg-rows))

; COLLECTION DEFINITIONS
(define initialize-grid
  (lambda ()
    (let init ((l grid-coordinates))
      (cond
        ((null? l)
         (quote ()))
        (else (cons
               (cons (car l) (list unvisited))
               (init (cdr l))))))))
(define grid '())
(set! grid (initialize-grid))
(define initialize-horiz-seg
  (lambda ()
    (let init ((l horiz-seg-coordinates))
      (cond
        ((null? l)
         (quote ()))
        (else
         (cond
           ((or (= (car (car l)) 0)
                (= (car (car l)) (- num-horiz-seg-rows 1)))
            (cons (cons (car l) (list blocked))
                  (init (cdr l))))
           (else
            (cons (cons (car l) (list clear))
                  (init (cdr l))))))))))
(define horiz-seg '())
(set! horiz-seg (initialize-horiz-seg))

(define initialize-vert-seg
  (lambda ()
    (let init ((l vert-seg-coordinates))
      (cond
        ((null? l)
         (quote ()))
        (else
         (cond
           ((or (= (car (cdr (car l))) 0)
                (= (car (cdr (car l))) (- num-vert-seg-rows 1)))
            (cons (cons (car l) (list blocked))
                  (init (cdr l))))
           (else
            (cons (cons (car l) (list clear))
                  (init (cdr l))))))))))
(define vert-seg '())
(set! vert-seg (initialize-vert-seg))

; COLLECTION ELEMENT RETRIEVAL

; Content of Absolute Coordinates
(define coordinate
  (lambda (array row col)
    (cond
      ((and (= (car (car (car array))) row)
            (= (car (cdr (car (car array)))) col))
       (car (cdr (car array))))
      (else
       (coordinate (cdr array) row col)))))

(define coordinate-set
  (lambda (array row col val)
    (cond
      ((and (= (car (car (car array))) row)
            (= (car (cdr (car (car array)))) col))
       (cons (cons (car (car array)) (list val)) (cdr array)))
      (else
       (cons (car array) (coordinate-set (cdr array) row col val))))))

; Relative Coordinate
(define rel-coordinate
  (lambda (dir row col)
    (cond
      ((eq? dir up) (list (- row 1) col))
      ((eq? dir right) (list row (+ col 1)))
      ((eq? dir down) (list (+ row 1) col))
      ((eq? dir left) (list row (- col 1)))
      (else (error "ERROR: rel-coordinate")))))


; Relative Grid-to-Segment
(define rel-seg
  (lambda (dir row col)
    (cond
      ((eq? dir up) (coordinate horiz-seg row col))
      ((eq? dir right) (coordinate vert-seg row (+ col 1)))
      ((eq? dir down) (coordinate horiz-seg (+ row 1) col))
      ((eq? dir left) (coordinate vert-seg row col))
      (else (error "ERROR: relative-seg")))))

; BRANCH-AND-BOUND

; constant representing length of path that never reaches node (0,0)
(define infinity 999999)

; list-of-lists-of-coordinates (which are also lists) used in branch-and-bound
(define paths-home '())
(set! paths-home (list (list current)))

(define contains-coordinate?
  (lambda (c l)
    (cond
      ((null? l) #f)
      ((let one-path ((p (car l)))
         (cond
           ((null? p) #f)
           ((and (eq? (car c) (car (car p)))
                 (eq? (car (cdr c)) (car (cdr (car p))))) #t)
           (else (one-path (cdr p))))) #t)
      (else
       (contains-coordinate? c (cdr l))))))

(define path-size 0)
(set! path-size 1)

(define one-direction
  (lambda (dir p) (begin
    (cond
      ; check to see if path is clear in direction 'dir'
      ((eq? clear (rel-seg dir
                           (car (last (car p)))
                           (car (cdr (last (car p))))))
        ; check to see if the next node is already in 'paths-home'
       (cond
         ((not (contains-coordinate? (rel-coordinate dir
                                                     (car (last (car p)))
                                                     (car (cdr (last (car p)))))
                                     paths-home))
          ; if 'paths-home' does not already contain the next coordinate,
          ; add the next node to the path and add the updated
          ; path the 'paths-home' list
          (set! paths-home
                (append
                 (append (rel-coordinate dir
                                         (car (last (car p)))
                                         (car (cdr (last (car p)))))
                         (car p))
                 paths-home))))))
    ; if we have found a clear path home, we return #t
    (cond
      ((and (eq? clear (rel-seg dir
                                (car (last (car p)))
                                (car (cdr (last (car p))))))
            (equal? '(0 0) (rel-coordinate dir
                                           (car (last (car p)))
                                           (car (cdr (last (car p))))))) #t)
      (else
       ; if 'dir' is not the last direction in 'directions', recur over 'one-direction'
       ; using the next direction in 'directions'
       (cond
         ((not (eq? dir left))
          (one-direction (after dir directions) p))
         ; we did not find a path home, so we return #f)
         (else #f)))))))

(define one-tree-level
  (lambda (p)
    (cond
      ((null? p) #f)
      (else
       (cond
         ((one-direction up p) #t)
         (else (one-tree-level (cdr p))))))))

(define remove-dead-ends
  (lambda (l)
    (when (not (null? l))
      (when (< (length (car l)) path-size)
        (set! paths-home (remove (car l) paths-home)))
      (remove-dead-ends (cdr l)))))

(define branch-and-bound
  (lambda (p) ; NOTE: p must *always* be sent paths-home
    (cond
      ((one-tree-level p) (last paths-home))
      (else (begin
              (set! path-size (+ 1 path-size))
              (remove-dead-ends p)
              (branch-and-bound paths-home))))))

(define test-rel-seg
  (lambda ()
    (let test ((l grid-coordinates))
      (when (not (null? l))
        (display "(")(display (car (car l)))(display ",")(display (car (cdr (car l))))
        (display ") ")
        (let _test ((dir up))
          (begin
            (display dir)(display ":")
            (display (rel-seg dir (car (car l)) (car (cdr (car l)))))(display " ")
            (when (not (eq? dir left))
              (_test (after dir directions)))))
        (newline)
        (test (cdr l))))))

(define set-blocks
  (lambda (horiz vert)
    (cond
      ((or (not (list? horiz)) (not (list? vert)))
       (error "ERROR: must supply lists of coordinates"))
      (else
       (begin
         (let set-h ((h horiz))
           (cond
             ((not (null? h))
              (begin
                (set! horiz-seg
                      (coordinate-set horiz-seg
                                      (car (car h))
                                      (car (cdr (car h)))
                                      blocked))
                (set-h (cdr h))))))
         (let set-v ((v vert))
           (cond
             ((not (null? v))
              (begin
                (set! vert-seg
                      (coordinate-set vert-seg
                                      (car (car v))
                                      (car (cdr (car v)))
                                      blocked))
                (set-v (cdr v)))))))))))

(define reset-segments
  (lambda ()
    (begin
      (set! horiz-seg (initialize-horiz-seg))
      (set! vert-seg (initialize-vert-seg)))))

; TEST CONDITIONS

; NOTES:
; - To run a test, simply enter (test1) or (test2) or ...
;   into the REPL (read-eval-print-loop)
; - X denotes the robot's current position
; - ^/>/v/< denote directional arrows along the path home
;   that the robot chooses according to the branch-and-bound function
; - $ denotes the end of the route home

;    0   1   2   3   4   5
;  +###+###+###+###+###+###+
; 0# $ | < |   |   |   |   #
;  +###+---+###+---+---+---+
; 1#   # ^ | < |   |   |   #
;  +---+###+---+---+---+---+
; 2#   |   # ^ |   |   |   #
;  +---+---+---+---+---+---+
; 3# X | > | ^ |   |   |   #
;  +---+---+---+---+---+---+
; 4#   |   |   |   |   |   #
;  +---+---+---+---+---+---+
; 5#   |   |   |   |   |   #
;  +---+---+---+---+---+---+
; 6#   |   |   |   |   |   #
;  +###+###+###+###+###+###+
(define test1
  (lambda ()
    (begin
      (reset-segments)
      (set-blocks '((1 0) (1 2) (2 1)) ; horiz-segs
                  '((1 1) (2 2))) ; vert-segs
      (set! current '(3 0))
      (set! paths-home (list (list current)))
      (branch-and-bound paths-home))))

;    0   1   2   3   4   5
;  +###+###+###+###+###+###+
; 0# $ |   |   |   |   |   #
;  +---+###+###+---+---+---+
; 1# ^ #   |   |   |   |   #
;  +---+###+---+---+---+---+
; 2# ^ | < #   |   |   |   #
;  +###+---+###+###+---+---+
; 3# > | ^ #   |   |   |   #
;  +---+###+---+---+---+---+
; 4# ^ | < | X |   |   |   #
;  +---+---+---+---+---+---+
; 5#   |   |   |   |   |   #
;  +---+---+---+---+---+---+
; 6#   |   |   |   |   |   #
;  +###+###+###+###+###+###+
(define test2
  (lambda ()
    (begin
      (reset-segments)
      (set-blocks '((1 1) (1 2) (2 1) (3 0) (3 2) (3 3) (4 1)) ; horiz-segs
                  '((1 1) (2 2) (3 2))) ; vert-segs
      (set! current '(4 2))
      (set! paths-home (list (list current)))
      (branch-and-bound paths-home))))

;    0   1   2   3   4   5
;  +###+###+###+###+###+###+
; 0# $ #   |   |   |   #   #
;  +---+---+###+###+---+---+
; 1# ^ #   |   |   #   #   #
;  +---+###+###+###+---+---+
; 2# ^ | < | < #   |   #   #
;  +###+###+---+---+###+---+
; 3# > | v # ^ #   #   |   #
;  +---+---+---+---+---+---+
; 4# ^ # > | ^ #   #   #   #
;  +---+###+###+---+---+---+
; 5# ^ # v | < | < #   #   #
;  +---+---+###+---+---+###+
; 6# ^ | < #   | ^ | < | X #
;  +###+###+###+###+###+###+
(define test3
  (lambda ()
    (begin
      (reset-segments)
      (set-blocks '((1 2) (1 3)
                    (2 1) (2 2) (2 3)
                    (3 0) (3 1) (3 4)
                    (5 1) (5 2)
                    (6 2) (6 5)) ; horiz segs
                  '((0 1) (0 5)
                    (1 1) (1 4) (1 5)
                    (2 3) (2 5)
                    (3 2) (3 3) (3 4)
                    (4 1) (4 3) (4 4) (4 5)
                    (5 1) (5 4) (5 5)
                    (6 2))) ; vert segs
      (set! current '(6 5))
      (set! paths-home (list (list current)))
      (branch-and-bound paths-home))))

; ADD MORE TESTS HERE AS DESIRED
