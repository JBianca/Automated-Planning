;; Two robot with different capacity:
;; alpha_bot_large, robot that process alpha artifact with capacity three;
;; beta_bot_large, robot that process beta artifact with capacity three.

(define (problem interplanetary_museum_vault_problem2)
  (:domain interplanetary_museum_vault)

  (:objects
    alpha_bot_large - alpha_robot
    beta_bot_large - beta_robot
    
    entrance hall_alpha hall_beta cryo_chamber stasis_lab pod_area - location
    
    pod1 pod2 - pod  
    
    artifact_a1 artifact_a2 artifact_a3 - artifact  
    artifact_b1 artifact_b2 artifact_b3 - artifact  
  )

  (:init
    ;; Robot locations
    (at alpha_bot_large entrance)
    (at beta_bot_large entrance)
    
    ;; Robot sealing
    (not (sealed alpha_bot_large))
    (not (sealed beta_bot_large))
    
    ;; Robot capacities
    (capacity_level_0 alpha_bot_large)
    (capacity_level_0 beta_bot_large)
    (max_capacity_3 alpha_bot_large)  
    (max_capacity_3 beta_bot_large)  
    (has_capacity alpha_bot_large)
    (has_capacity beta_bot_large)
    
    ;; Artifact locations
    (at_artifact artifact_a1 hall_alpha)
    (at_artifact artifact_a2 hall_alpha)
    (at_artifact artifact_a3 hall_alpha)
    (at_artifact artifact_b1 hall_beta)
    (at_artifact artifact_b2 hall_beta)
    (at_artifact artifact_b3 hall_beta)
    
    ;; Artifact types
    (is_alpha_art artifact_a1) (is_alpha_art artifact_a2) (is_alpha_art artifact_a3)
    (is_beta_art artifact_b1) (is_beta_art artifact_b2) (is_beta_art artifact_b3)
    
    ;; Artifact states
    (not (cooled artifact_a1)) (not (cooled artifact_a2)) (not (cooled artifact_a3))
    (not (antivibration_on artifact_b1)) (not (antivibration_on artifact_b2)) (not (antivibration_on artifact_b3))
    (not (ready_for_stasis artifact_a1)) (not (ready_for_stasis artifact_a2)) (not (ready_for_stasis artifact_a3))
    (not (ready_for_stasis artifact_b1)) (not (ready_for_stasis artifact_b2)) (not (ready_for_stasis artifact_b3))
    
    ;; Pod setup 
    (pod_at pod1 pod_area)
    (pod_at pod2 pod_area)
    (pod_free pod1)
    (pod_free pod2)
    (pod_slot_available)   
    (pods_used_0)         
    (not (pods_used_1))
    (not (pods_used_2))
    
    ;; Environment
    (stable entrance)
    (stable hall_alpha)
    (stable cryo_chamber)
    (stable stasis_lab)
    (stable pod_area)
    (stable maintenance_tunnel)
    (not (stable hall_beta))
  )

  (:goal (and
    (at_artifact artifact_a1 stasis_lab)
    (at_artifact artifact_a2 stasis_lab)
    (at_artifact artifact_a3 stasis_lab)
    (at_artifact artifact_b1 stasis_lab)
    (at_artifact artifact_b2 stasis_lab)
    (at_artifact artifact_b3 stasis_lab)
  ))
)