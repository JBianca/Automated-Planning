(define (domain museum_vault)
  (:requirements :strips :typing :durative-actions) 
  
  (:types
    artifact location pod - object
    robot - object
    alpha_robot - robot
    beta_robot - robot
  )
  
  (:predicates
    ;; Robot location and state
    (at_robot ?r - robot ?x - location)
    (sealed ?r - robot)
    (unsealed ?r - robot)
    
    ;; Artifact location and state
    (at_artifact ?o - artifact ?x - location)
    (carrying ?r - robot ?o - artifact)
    (cooled ?o - artifact)
    (uncooled ?o - artifact)
    (antivibration_on ?o - artifact)
    (antivibration_off ?o - artifact)
    (ready_for_stasis ?o - artifact)
    
    ;; Artifact types
    (is_alpha_art ?o - artifact)
    (is_beta_art ?o - artifact)
    
    ;; Pod state
    (pod_at ?p - pod ?x - location)
    (pod_free ?p - pod)
    (pod_occupied ?o - artifact ?p - pod)
    
    ;; Environment
    (stable ?x - location)
    (unstable ?x - location)
    
    ;; Location types for constraints
    (is_hall_alpha ?x - location)
    (is_hall_beta ?x - location)
    (is_cryo_chamber ?x - location)
    (is_stasis_lab ?x - location)
    (is_pod_area ?x - location)
  )

  ;; --- DURATIVE MOVEMENT ---
  (:durative-action move
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 4)
    :condition (and
        (at start (at_robot ?r ?from))
        (at start (stable ?to))
        (at start (sealed ?r)))
    :effect (and
        (at end (at_robot ?r ?to))
        (at start (not (at_robot ?r ?from)))))
  
  (:durative-action seal
    :parameters (?r - robot)
    :duration (= ?duration 2)
    :condition (at start (unsealed ?r))
    :effect (and
        (at end (sealed ?r))
        (at start (not (unsealed ?r)))))
  
  (:durative-action unseal
    :parameters (?r - robot)
    :duration (= ?duration 2)
    :condition (at start (sealed ?r))
    :effect (and
        (at end (unsealed ?r))
        (at start (not (sealed ?r)))))
  
  ;; --- Alpha artifact workflow ---
  (:durative-action pick_hall_alpha
    :parameters (?r - alpha_robot ?a - artifact ?loc - location)
    :duration (= ?duration 3)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (at_artifact ?a ?loc))
        (at start (is_alpha_art ?a))
        (at start (uncooled ?a))
        (at start (is_hall_alpha ?loc))) 
    :effect (and
        (at end (carrying ?r ?a))
        (at start (not (at_artifact ?a ?loc)))))
  
  (:durative-action drop_at_cryo
    :parameters (?r - robot ?a - artifact ?loc - location)
    :duration (= ?duration 5)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (carrying ?r ?a))
        (at start (is_alpha_art ?a))
        (at start (uncooled ?a))
        (at start (is_cryo_chamber ?loc))) 
    :effect (and
        (at end (at_artifact ?a ?loc))
        (at start (not (carrying ?r ?a)))))
  
  (:durative-action cool_artifact
    :parameters (?r - robot ?a - artifact ?loc - location)
    :duration (= ?duration 6)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (at_artifact ?a ?loc))
        (at start (is_alpha_art ?a))
        (at start (uncooled ?a))
        (at start (is_cryo_chamber ?loc))) 
    :effect (and
        (at end (cooled ?a))
        (at end (ready_for_stasis ?a))
        (at start (not (uncooled ?a)))))
  
  (:durative-action pick_from_cryo
    :parameters (?r - robot ?a - artifact ?loc - location)
    :duration (= ?duration 3)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (at_artifact ?a ?loc))
        (at start (is_alpha_art ?a))
        (at start (cooled ?a))
        (at start (ready_for_stasis ?a))
        (at start (is_cryo_chamber ?loc))) 
    :effect (and
        (at end (carrying ?r ?a))
        (at start (not (at_artifact ?a ?loc)))))
  
  ;; --- Beta artifact workflow ---
  (:durative-action pick_hall_beta
    :parameters (?r - beta_robot ?a - artifact ?loc - location)
    :duration (= ?duration 3)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (at_artifact ?a ?loc))
        (at start (is_beta_art ?a))
        (at start (stable ?loc))
        (at start (antivibration_off ?a))
        (at start (is_hall_beta ?loc)))  
    :effect (and
        (at end (carrying ?r ?a))
        (at start (not (at_artifact ?a ?loc)))))
  
  (:durative-action drop_at_pod
    :parameters (?r - robot ?a - artifact ?p - pod ?loc - location)
    :duration (= ?duration 5)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (carrying ?r ?a))
        (at start (is_beta_art ?a))
        (at start (pod_free ?p))
        (at start (pod_at ?p ?loc))
        (at start (antivibration_off ?a))
        (at start (is_pod_area ?loc)))  
    :effect (and
        (at end (pod_occupied ?a ?p))
        (at start (not (carrying ?r ?a)))
        (at start (not (pod_free ?p)))))
  
  (:durative-action activate_antivibration
    :parameters (?r - robot ?a - artifact ?p - pod ?loc - location)
    :duration (= ?duration 6)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (pod_occupied ?a ?p))
        (at start (is_beta_art ?a))
        (at start (antivibration_off ?a))
        (at start (is_pod_area ?loc)))  
    :effect (and
        (at end (antivibration_on ?a))
        (at end (ready_for_stasis ?a))
        (at start (not (antivibration_off ?a)))))
  
  (:durative-action pick_from_pod
    :parameters (?r - robot ?a - artifact ?p - pod ?loc - location)
    :duration (= ?duration 3)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (pod_occupied ?a ?p))
        (at start (is_beta_art ?a))
        (at start (antivibration_on ?a))
        (at start (ready_for_stasis ?a))
        (at start (is_pod_area ?loc)))  
    :effect (and
        (at end (carrying ?r ?a))
        (at end (pod_free ?p))
        (at start (not (pod_occupied ?a ?p)))))
  
  ;; --- Final delivery ---
  (:durative-action deliver_to_stasis
    :parameters (?r - robot ?a - artifact ?loc - location)
    :duration (= ?duration 5)
    :condition (and
        (at start (at_robot ?r ?loc))
        (at start (carrying ?r ?a))
        (at start (ready_for_stasis ?a))
        (at start (is_stasis_lab ?loc)))  
    :effect (and
        (at end (at_artifact ?a ?loc))
        (at start (not (carrying ?r ?a)))))
  
  (:durative-action stabilize
    :parameters (?loc - location)
    :duration (= ?duration 2)
    :condition (at start (unstable ?loc))
    :effect (and
        (at end (stable ?loc))
        (at start (not (unstable ?loc)))))
)