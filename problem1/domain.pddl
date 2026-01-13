(define (domain interplanetary_museum_vault)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    robot artifact location pod - object
  )

  (:predicates
    ;; Robot states
    (at ?r - robot ?loc - location) ;; robot r at location loc
    (carrying ?r - robot ?a - artifact) ;; robot r carrying artifact a
    (empty-handed ?r - robot)  ;; robot r empty handed or not         
    (sealed ?r - robot)  ;; robot r in seal mode or not

    ;; Artifact states
    (at_artifact ?a - artifact ?loc - location) ;; artifact a at location loc
    (cooled ?a - artifact)  ;; artifact a cooled or not            
    (antivibration_on ?a - artifact) ;; artifact a in antivibration mode or not
    (ready_for_stasis ?a - artifact)  ;; artifact a ready for stasis-lab or not
    
    ;; Artifact types
    (is_alpha_art ?a - artifact)  ;; is artifact a of type alpha
    (is_beta_art ?a - artifact)  ;; is artifact a of type beta

    ;; Pod predicates
    (pod_at ?p - pod ?loc - location)  ;; pod p at location loc (pod area)
    (pod_free ?p - pod)  ;; pod p free or not
    (pod_occupied ?a - artifact ?p - pod)  ;; pod p occupied or not
    
    ;; Environmental
    (stable ?loc - location)  ;; location loc stable or not
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
    :parameters (?r - robot ?a - artifact)
    :precondition (and 
        (at ?r hall_alpha) 
        (at_artifact ?a hall_alpha) 
        (is_alpha_art ?a) 
        (empty-handed ?r)              
        (not (cooled ?a))
    )
    :effect (and 
        (carrying ?r ?a) 
        (not (at_artifact ?a hall_alpha))
        (not (empty-handed ?r))        
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
        (empty-handed ?r)              
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
        (empty-handed ?r)              
    )
    :effect (and 
        (carrying ?r ?a) 
        (not (at_artifact ?a cryo_chamber))
        (not (empty-handed ?r))
    )
  )

  ;; --- Hall beta workflow ---

  ;; Pick beta artifact from hall beta
  (:action pick_hall_beta
    :parameters (?r - robot ?a - artifact)
    :precondition (and 
        (at ?r hall_beta) 
        (at_artifact ?a hall_beta) 
        (is_beta_art ?a) 
        (stable hall_beta)
        (empty-handed ?r)              
        (not (antivibration_on ?a))
    )
    :effect (and 
        (carrying ?r ?a) 
        (not (at_artifact ?a hall_beta))
        (not (empty-handed ?r))
    )
  )

  ;; Drop beta artifact at pod area
  (:action drop_at_pod
    :parameters (?r - robot ?a - artifact ?p - pod)
    :precondition (and 
        (at ?r pod_area) 
        (carrying ?r ?a) 
        (is_beta_art ?a) 
        (pod_at ?p pod_area) 
        (pod_free ?p)
        (not (antivibration_on ?a))
    )
    :effect (and 
        (pod_occupied ?a ?p) 
        (not (carrying ?r ?a)) 
        (not (pod_free ?p))
        (empty-handed ?r)             
    )
  )

  ;; Activate antivibration mode on beta artifact
  (:action activate_antivibration
    :parameters (?r - robot ?a - artifact ?p - pod)
    :precondition (and 
        (at ?r pod_area) 
        (pod_occupied ?a ?p) 
        (pod_at ?p pod_area) 
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
        (pod_at ?p pod_area) 
        (antivibration_on ?a) 
        (ready_for_stasis ?a)
        (empty-handed ?r)              
    )
    :effect (and 
        (carrying ?r ?a) 
        (pod_free ?p) 
        (not (pod_occupied ?a ?p))
        (not (empty-handed ?r))
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
        (empty-handed ?r)             
    )
  )

  ;; Stabilize hall_beta
  (:action stabilize
    :parameters ()
    :precondition (not (stable hall_beta))
    :effect (stable hall_beta)
  )
)