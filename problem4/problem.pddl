(define (problem museum_vault_problem)
  (:domain museum_vault)
  
  (:objects
    robot_alpha - alpha_robot
    robot_beta - beta_robot
    
    entrance hall_alpha hall_beta cryo_chamber stasis_lab pod_area - location
    
    pod1 pod2 - pod
    
    artifact_a1 artifact_a2 artifact_a3 - artifact
    artifact_b1 artifact_b2 artifact_b3 - artifact
  )
  
  (:init
    ;; Robot locations
    (at_robot robot_alpha entrance)
    (at_robot robot_beta entrance)
    
    ;; Robot sealing
    (unsealed robot_alpha)
    (unsealed robot_beta)
    
    ;; Artifact locations
    (at_artifact artifact_a1 hall_alpha)
    (at_artifact artifact_a2 hall_alpha)
    (at_artifact artifact_a3 hall_alpha)
    (at_artifact artifact_b1 hall_beta)
    (at_artifact artifact_b2 hall_beta)
    (at_artifact artifact_b3 hall_beta)
    
    ;; Artifact types
    (is_alpha_art artifact_a1)
    (is_alpha_art artifact_a2)
    (is_alpha_art artifact_a3)
    (is_beta_art artifact_b1)
    (is_beta_art artifact_b2)
    (is_beta_art artifact_b3)
    
    ;; Artifact states
    (uncooled artifact_a1)
    (uncooled artifact_a2)
    (uncooled artifact_a3)
    (antivibration_off artifact_b1)
    (antivibration_off artifact_b2)
    (antivibration_off artifact_b3)
    
    ;; Pod setup
    (pod_at pod1 pod_area)
    (pod_at pod2 pod_area)
    (pod_free pod1)
    (pod_free pod2)
    
    ;; Environment
    (stable entrance)
    (stable hall_alpha)
    (stable cryo_chamber)
    (stable stasis_lab)
    (stable pod_area)
    (unstable hall_beta)
    
    ;; Location types for constraints
    (is_hall_alpha hall_alpha)
    (is_hall_beta hall_beta)
    (is_cryo_chamber cryo_chamber)
    (is_stasis_lab stasis_lab)
    (is_pod_area pod_area)
  )
  
  (:goal (and
    (at_artifact artifact_a1 stasis_lab)
    (at_artifact artifact_a2 stasis_lab)
    (at_artifact artifact_a3 stasis_lab)
    (at_artifact artifact_b1 stasis_lab)
    (at_artifact artifact_b2 stasis_lab)
    (at_artifact artifact_b3 stasis_lab)
  ))
  
  (:metric minimize (total-time))
)