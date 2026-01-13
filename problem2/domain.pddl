(define (domain interplanetary_museum_vault)
  (:requirements :strips :typing :negative-preconditions :equality)

  (:types
    artifact location pod - object
    robot - object
    alpha_robot - robot
    beta_robot - robot
    capacity_level - object
  )

  (:predicates
    ;; Basic predicates
    (at ?r - robot ?loc - location)
    (carrying ?r - robot ?a - artifact)
    (sealed ?r - robot)
    
    ;; Capacity tracking
    (capacity_level_0 ?r - robot)
    (capacity_level_1 ?r - robot)
    (capacity_level_2 ?r - robot)
    (capacity_level_3 ?r - robot)
    (max_capacity_1 ?r - robot)
    (max_capacity_2 ?r - robot)
    (max_capacity_3 ?r - robot)
    (has_capacity ?r - robot)
    
    ;; Artifact states
    (at_artifact ?a - artifact ?loc - location)
    (cooled ?a - artifact)
    (antivibration_on ?a - artifact)
    (ready_for_stasis ?a - artifact)
    
    ;; Artifact types
    (is_alpha_art ?a - artifact)
    (is_beta_art ?a - artifact)
    
    ;; Pod predicates 
    (pod_at ?p - pod ?loc - location)
    (pod_free ?p - pod)
    (pod_occupied ?a - artifact ?p - pod)
    (pod_slot_available)          ; True if <2 pods occupied
    (pods_used_0)                 ; 0 pods in use
    (pods_used_1)                 ; 1 pod in use
    (pods_used_2)                 ; 2 pods in use (max)
    
    ;; Environmental
    (stable ?loc - location)
  )

  ;; Move robot between locations
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and 
        (at ?r ?from)
        (stable ?to)
        (sealed ?r)
        (not (= ?from ?to))
    )
    :effect (and (at ?r ?to) (not (at ?r ?from)))
  )

  ;; Activate seal/unseal mode on robot
  (:action seal
    :parameters (?r - robot)
    :precondition (not (sealed ?r))
    :effect (sealed ?r)
  )

  (:action unseal
    :parameters (?r - robot)
    :precondition (sealed ?r)
    :effect (not (sealed ?r))
  )

  ;; --- Hall alpha workflow ---

  ;; Pick alpha artifact from hall alpha
  (:action pick_hall_alpha
    :parameters (?r - alpha_robot ?a - artifact)
    :precondition (and 
        (at ?r hall_alpha)
        (at_artifact ?a hall_alpha)
        (is_alpha_art ?a)
        (has_capacity ?r)
        (not (cooled ?a))
    )
    :effect (and 
        (carrying ?r ?a)
        (not (at_artifact ?a hall_alpha))
        (when (capacity_level_0 ?r) (and (not (capacity_level_0 ?r)) (capacity_level_1 ?r) (when (max_capacity_1 ?r) (not (has_capacity ?r)))))
        (when (capacity_level_1 ?r) (and (not (capacity_level_1 ?r)) (capacity_level_2 ?r) (when (max_capacity_2 ?r) (not (has_capacity ?r)))))
        (when (capacity_level_2 ?r) (and (not (capacity_level_2 ?r)) (capacity_level_3 ?r) (not (has_capacity ?r))))
    )
  )

  ;; Drop alpha artifact at the cryo-chamber
  (:action drop_at_cryo
    :parameters (?r - robot ?a - artifact)
    :precondition (and 
        (at ?r cryo_chamber)
        (carrying ?r ?a)
        (is_alpha_art ?a)
        (not (cooled ?a))
    )
    :effect (and 
        (at_artifact ?a cryo_chamber)
        (not (carrying ?r ?a))
        (when (capacity_level_1 ?r) (and (not (capacity_level_1 ?r)) (capacity_level_0 ?r) (has_capacity ?r)))
        (when (capacity_level_2 ?r) (and (not (capacity_level_2 ?r)) (capacity_level_1 ?r) (has_capacity ?r)))
        (when (capacity_level_3 ?r) (and (not (capacity_level_3 ?r)) (capacity_level_2 ?r) (has_capacity ?r)))
    )
  )

  ;; Cool the alpha artifact 
  (:action cool
    :parameters (?r - robot ?a - artifact)
    :precondition (and 
        (at ?r cryo_chamber)
        (at_artifact ?a cryo_chamber)
        (is_alpha_art ?a)
        (not (cooled ?a))
    )
    :effect (and 
        (cooled ?a)
        (ready_for_stasis ?a)
    )
  )

  ;; Pick alpha artifact from cryo-chamber
  (:action pick_from_cryo
    :parameters (?r - robot ?a - artifact)
    :precondition (and 
        (at ?r cryo_chamber)
        (at_artifact ?a cryo_chamber)
        (is_alpha_art ?a)
        (cooled ?a)
        (ready_for_stasis ?a)
        (has_capacity ?r)
    )
    :effect (and 
        (carrying ?r ?a)
        (not (at_artifact ?a cryo_chamber))
        (when (capacity_level_0 ?r) (and (not (capacity_level_0 ?r)) (capacity_level_1 ?r) (when (max_capacity_1 ?r) (not (has_capacity ?r)))))
        (when (capacity_level_1 ?r) (and (not (capacity_level_1 ?r)) (capacity_level_2 ?r) (when (max_capacity_2 ?r) (not (has_capacity ?r)))))
        (when (capacity_level_2 ?r) (and (not (capacity_level_2 ?r)) (capacity_level_3 ?r) (not (has_capacity ?r))))
    )
  )

  ;; --- Hall beta workflow ---

  ;; Pick beta artifact from hall beta
  (:action pick_hall_beta
    :parameters (?r - beta_robot ?a - artifact)
    :precondition (and 
        (at ?r hall_beta)
        (at_artifact ?a hall_beta)
        (is_beta_art ?a)
        (stable hall_beta)
        (has_capacity ?r)
        (not (antivibration_on ?a))
    )
    :effect (and 
        (carrying ?r ?a)
        (not (at_artifact ?a hall_beta))
        (when (capacity_level_0 ?r) (and (not (capacity_level_0 ?r)) (capacity_level_1 ?r) (when (max_capacity_1 ?r) (not (has_capacity ?r)))))
        (when (capacity_level_1 ?r) (and (not (capacity_level_1 ?r)) (capacity_level_2 ?r) (when (max_capacity_2 ?r) (not (has_capacity ?r)))))
        (when (capacity_level_2 ?r) (and (not (capacity_level_2 ?r)) (capacity_level_3 ?r) (not (has_capacity ?r))))
    )
  )

  ;; Drop beta artifact at pod area
  (:action drop_at_pod
    :parameters (?r - robot ?a - artifact ?p - pod)
    :precondition (and 
        (at ?r pod_area)
        (carrying ?r ?a)
        (is_beta_art ?a)
        (pod_free ?p)
        (pod_at ?p pod_area)
        (not (antivibration_on ?a))
        (pod_slot_available)         
    )
    :effect (and 
        (pod_occupied ?a ?p)
        (not (carrying ?r ?a))
        (not (pod_free ?p))
        (when (pods_used_0)
          (and 
            (not (pods_used_0))
            (pods_used_1)
          )
        )
        (when (pods_used_1)
          (and 
            (not (pods_used_1))
            (pods_used_2)
            (not (pod_slot_available))
          )
        )
        ;; Update robot capacity
        (when (capacity_level_1 ?r) (and (not (capacity_level_1 ?r)) (capacity_level_0 ?r) (has_capacity ?r)))
        (when (capacity_level_2 ?r) (and (not (capacity_level_2 ?r)) (capacity_level_1 ?r) (has_capacity ?r)))
        (when (capacity_level_3 ?r) (and (not (capacity_level_3 ?r)) (capacity_level_2 ?r) (has_capacity ?r)))
    )
  )

  ;; Activate antivibration mode on beta artifact
  (:action activate_antivibration
    :parameters (?r - robot ?a - artifact ?p - pod)
    :precondition (and 
        (at ?r pod_area)
        (pod_occupied ?a ?p)
        (is_beta_art ?a)
        (not (antivibration_on ?a))
    )
    :effect (and 
        (antivibration_on ?a)
        (ready_for_stasis ?a)
    )
  )

  ;; Pick beta artifact from pod area
  (:action pick_from_pod
    :parameters (?r - robot ?a - artifact ?p - pod)
    :precondition (and 
        (at ?r pod_area)
        (pod_occupied ?a ?p)
        (antivibration_on ?a)
        (ready_for_stasis ?a)
        (has_capacity ?r)
    )
    :effect (and 
        (carrying ?r ?a)
        (pod_free ?p)
        (not (pod_occupied ?a ?p))
        (when (pods_used_2)
          (and 
            (not (pods_used_2))
            (pods_used_1)
            (pod_slot_available)      
          )
        )
        (when (pods_used_1)
          (and 
            (not (pods_used_1))
            (pods_used_0)
          )
        )
        ;; Update robot capacity
        (when (capacity_level_0 ?r) (and (not (capacity_level_0 ?r)) (capacity_level_1 ?r) (when (max_capacity_1 ?r) (not (has_capacity ?r)))))
        (when (capacity_level_1 ?r) (and (not (capacity_level_1 ?r)) (capacity_level_2 ?r) (when (max_capacity_2 ?r) (not (has_capacity ?r)))))
        (when (capacity_level_2 ?r) (and (not (capacity_level_2 ?r)) (capacity_level_3 ?r) (not (has_capacity ?r))))
    )
  )

  ;; Deliver artifact to stasis-lab
  (:action deliver_to_stasis
    :parameters (?r - robot ?a - artifact)
    :precondition (and 
        (at ?r stasis_lab)
        (carrying ?r ?a)
        (ready_for_stasis ?a)
    )
    :effect (and 
        (at_artifact ?a stasis_lab)
        (not (carrying ?r ?a))
        (when (capacity_level_1 ?r) (and (not (capacity_level_1 ?r)) (capacity_level_0 ?r) (has_capacity ?r)))
        (when (capacity_level_2 ?r) (and (not (capacity_level_2 ?r)) (capacity_level_1 ?r) (has_capacity ?r)))
        (when (capacity_level_3 ?r) (and (not (capacity_level_3 ?r)) (capacity_level_2 ?r) (has_capacity ?r)))
    )
  )
  
  ; Stabilize hall_beta
  (:action stabilize
    :parameters ()
    :precondition (not (stable hall_beta))
    :effect (stable hall_beta)
  )
)