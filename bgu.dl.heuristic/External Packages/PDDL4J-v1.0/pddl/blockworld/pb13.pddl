(define (problem pb13)
   (:domain blocksworld)
   (:objects a b c d e f g h i j k l m)
   (:init (on-table a) (on-table b) (on-table c) (on-table d) (on-table e) 
          (on-table f) (on-table g) (on-table h) (on-table i) (on-table j)
          (on-table k)  (on-table l)  (on-table m)
          (clear a)  (clear b) (clear c) (clear d) (clear e) (clear j) 
          (clear f)  (clear g) (clear h) (clear i) (clear k) (clear l) 
          (clear m) (arm-empty))
   (:goal (and (on a b) (on b c) (on c d) (on d e) (on e f) (on f g)
               (on g h) (on h i) (on i j) (on j k) (on k l)(on l m)  )))


