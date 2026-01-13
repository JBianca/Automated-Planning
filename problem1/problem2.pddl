;; Single robot, six artifacts: 
;; artifact_a1, artifact_a2, artifact_a3, 
;; artifact_b1, artifact_b2, artifact_b3.

(define (problem interplanetary_museum_vault_fixed)
  (:domain interplanetary_museum_vault)

  (:objects
    curator - robot
    entrance hall_alpha hall_beta cryo_chamber stasis_lab pod_area - location
    pod1 pod2 - pod
    artifact_a1 artifact_a2 artifact_a3 artifact_b1 artifact_b2 artifact_b3 - artifact
  )

  (:init
    ;; Robot
    (at curator entrance)
    (not (sealed curator))
    (empty-handed curator) 
    (not (carrying curator artifact_a1))
    (not (carrying curator artifact_b1))
    (not (carrying curator artifact_a2))
    (not (carrying curator artifact_b2))
    (not (carrying curator artifact_a3))
    (not (carrying curator artifact_b3))


    ;; Artifacts
    (at_artifact artifact_a1 hall_alpha)
    (at_artifact artifact_b1 hall_beta)
    (at_artifact artifact_a2 hall_alpha)
    (at_artifact artifact_b2 hall_beta)
    (at_artifact artifact_a3 hall_alpha)
    (at_artifact artifact_b3 hall_beta)
    
    
    ;; Types
    (is_alpha_art artifact_a1)
    (is_beta_art artifact_b1)
    (is_alpha_art artifact_a2)
    (is_beta_art artifact_b2)
    (is_alpha_art artifact_a3)
    (is_beta_art artifact_b3)
    
    ;; States
    (not (cooled artifact_a1))
    (not (cooled artifact_a2))
    (not (cooled artifact_a3))


    (not (antivibration_on artifact_b1))
    (not (antivibration_on artifact_b2))
    (not (antivibration_on artifact_b3))


    (not (ready_for_stasis artifact_a1))
    (not (ready_for_stasis artifact_b1))
    (not (ready_for_stasis artifact_a2))
    (not (ready_for_stasis artifact_b2))
    (not (ready_for_stasis artifact_a3))
    (not (ready_for_stasis artifact_b3))
    
    ;; Pods
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
    (not (stable hall_beta))
  )

  (:goal (and
    (at_artifact artifact_a1 stasis_lab)
    (at_artifact artifact_b1 stasis_lab)
    (at_artifact artifact_a2 stasis_lab)
    (at_artifact artifact_b2 stasis_lab)
    (at_artifact artifact_a3 stasis_lab)
    (at_artifact artifact_b3 stasis_lab)
  ))
)
