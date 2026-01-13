;; Single robot, two artifacts: artifact_a, artifact_b.

(define (problem interplanetary_museum_vault_fixed)
  (:domain interplanetary_museum_vault)

  (:objects
    curator - robot
    entrance hall_alpha hall_beta cryo_chamber stasis_lab pod_area - location
    pod1 pod2 - pod
    artifact_a artifact_b - artifact
  )

  (:init
    ;; Robot
    (at curator entrance)
    (not (sealed curator))
    (empty-handed curator)  
    (not (carrying curator artifact_a))
    (not (carrying curator artifact_b))

    ;; Artifacts
    (at_artifact artifact_a hall_alpha)
    (at_artifact artifact_b hall_beta)
    
    ;; Types
    (is_alpha_art artifact_a)
    (is_beta_art artifact_b)
    
    ;; States
    (not (cooled artifact_a))
    (not (antivibration_on artifact_b))
    (not (ready_for_stasis artifact_a))
    (not (ready_for_stasis artifact_b))
    
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
    (at_artifact artifact_a stasis_lab)
    (at_artifact artifact_b stasis_lab)
  ))
)
