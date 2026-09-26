module fabrik_pipeline
  ! The chain pipeline: everything that happens to a chain between "the solver
  ! has produced joint positions" and "the adapter has orientations to pose a
  ! skeleton with".
  !
  ! This is the arithmetic the C++ adapter used to own. It is here so it can be
  ! tested by `fpm test` in seconds, under a sanitizer, instead of only through
  ! a GDScript test that needs a full godot-cpp build. The order is fixed and is
  ! the adapter's documented order:
  !
  !   1. solve                    (fabrik_core, unchanged)
  !   2. pole target              rigid rotation about the root-to-tip axis
  !   3. joint angle limits       per-joint projection of the sub-chain below
  !   4. residual                 re-measured, because 2 and 3 moved joints
  !   5. rotations                parallel-transported frame per bone
  !   6. smoothing                slerp in ROTATION space, then forward
  !                              kinematics with the declared lengths
  !
  ! Steps 2 and 3 cannot fight: a pole target is a rigid rotation about the
  ! root-to-tip axis and so changes no interior angle.
  use, intrinsic :: iso_c_binding, only: c_float, c_int
  use, intrinsic :: iso_fortran_env, only: real32
  use fabrik_status_codes
  use fabrik_geom
  implicit none
  private

  public :: measure_lengths
  public :: apply_pole
  public :: apply_joint_limits
  public :: residual_of
  public :: derive_rotations
  public :: smooth_rotations
  public :: blend_influence

contains

  pure function measure_lengths(joint_count, joints) result(lengths)
    integer, intent(in) :: joint_count
    real(real32), intent(in) :: joints(3, joint_count)
    ! Declared here, not as a dummy: `lengths` IS the function result.
    real(real32) :: lengths(joint_count - 1)
    integer :: i
    do i = 1, joint_count - 1
      lengths(i) = v_len(v_sub(joints(:, i + 1), joints(:, i)))
    end do
  end function measure_lengths

  pure function residual_of(joint_count, joints, target) result(residual)
    integer, intent(in) :: joint_count
    real(real32), intent(in) :: joints(3, joint_count), target(3)
    real(real32) :: residual
    residual = v_len(v_sub(joints(:, joint_count), target))
  end function residual_of

  pure subroutine apply_pole(joint_count, joints, pole_target)
    ! Rotate the intermediate joints about the root-to-tip axis so the bend faces
    ! the pole. Root and tip are fixed and every segment length is preserved,
    ! because the whole sub-chain above the root moves rigidly.
    integer, intent(in) :: joint_count
    real(real32), intent(inout) :: joints(3, joint_count)
    real(real32), intent(in) :: pole_target(3)
    real(real32) :: root(3), axis_full(3), axis(3), desired(3), current(3)
    real(real32) :: offset(3), turn(4), angle
    real(real32) :: best_length_squared
    integer :: i

    if (joint_count < 3) return
    if (.not. all_finite(pole_target)) return
    if (.not. all_finite(reshape(joints, [3 * joint_count]))) return
    if (v_len2(pole_target) <= EPS) return

    root = joints(:, 1)
    axis_full = v_sub(joints(:, joint_count), root)
    if (v_len2(axis_full) <= EPS) return
    axis = v_safe_unit(axis_full)

    ! Keep only the component of the pole perpendicular to the root-tip axis:
    ! that is the side the bend should face, and it is the only component a
    ! rotation around the chain axis can change.
    desired = v_sub(pole_target, root)
    desired = v_sub(desired, v_scale(axis, v_dot(desired, axis)))
    if (v_len2(desired) <= EPS) return
    desired = v_safe_unit(desired)

    ! The intermediate joint furthest from the axis defines the current bend
    ! side. A joint exactly on the axis gives no usable plane.
    current = 0.0_real32
    best_length_squared = EPS
    do i = 2, joint_count - 1
      offset = v_sub(joints(:, i), root)
      offset = v_sub(offset, v_scale(axis, v_dot(offset, axis)))
      if (v_len2(offset) > best_length_squared) then
        best_length_squared = v_len2(offset)
        current = offset
      end if
    end do
    if (v_len2(current) <= EPS) return
    current = v_safe_unit(current)

    angle = angle_about_axis(axis, current, desired)
    if (abs(angle) <= EPS) return

    turn = q_from_axis_angle(axis, angle)
    do i = 2, joint_count - 1
      joints(:, i) = v_add(root, q_xform(turn, v_sub(joints(:, i), root)))
    end do
  end subroutine apply_pole

  pure subroutine apply_joint_limits(joint_count, joints, joint_limits, limit_passes, &
      projections, violations)
    ! Project the chain into per-joint interior-angle limits.
    !
    ! Limits are given in DEGREES as the FLEXION angle between the incoming and
    ! outgoing segment: 0 is straight (the two segments are collinear), 180 is
    ! folded back on itself, so an elbow that may flex 0-150 degrees reads as
    ! (0, 150). This matches the adapter's implementation, which measures
    ! acos(incoming . outgoing); the adapter's own documentation claimed the
    ! opposite for as long as the feature existed, and porting the arithmetic
    ! here is what exposed it.
    ! "unlimited", and so does a joint past the end of the array.
    !
    ! The sub-chain BELOW a violating joint is rotated, not the one above it, so
    ! joints 1..i - and therefore an anchored root - stay exactly where they were
    ! and every segment length is preserved. The tip is what gives way, and the
    ! caller re-measures the residual to say so.
    !
    ! A limit at one joint can introduce a violation at the next, so this relaxes
    ! for a bounded number of passes and then REPORTS what it could not satisfy
    ! rather than implying the limits always hold.
    integer, intent(in) :: joint_count, limit_passes
    real(real32), intent(inout) :: joints(3, joint_count)
    real(real32), intent(in) :: joint_limits(:, :)
    integer, intent(out) :: projections, violations
    real(real32) :: minimum, maximum, theta, clamped
    real(real32) :: pivot(3), incoming(3), outgoing(3), in_dir(3), out_dir(3)
    real(real32) :: side(3), desired(3), correction(4)
    integer :: i, j, pass, passes, limit_count
    logical :: projected

    projections = 0
    violations = 0
    if (joint_count < 3) return
    if (.not. all_finite(reshape(joints, [3 * joint_count]))) return
    if (.not. all_finite(reshape(joint_limits, [size(joint_limits)]))) return
    limit_count = size(joint_limits, 2)
    if (limit_count < 3) return

    passes = max(1, limit_passes)
    do pass = 1, passes
      projected = .false.
      do i = 2, joint_count - 1
        if (i > limit_count) exit
        if (joint_limits(1, i) >= joint_limits(2, i)) cycle
        minimum = max(0.0_real32, min(180.0_real32, joint_limits(1, i)))
        maximum = max(0.0_real32, min(180.0_real32, joint_limits(2, i)))
        theta = interior_angle_degrees(joints(:, i - 1), joints(:, i), joints(:, i + 1))
        clamped = max(minimum, min(maximum, theta))
        if (abs(clamped - theta) <= 0.01_real32) cycle

        pivot = joints(:, i)
        incoming = v_sub(pivot, joints(:, i - 1))
        outgoing = v_sub(joints(:, i + 1), pivot)
        if (v_len2(incoming) <= EPS .or. v_len2(outgoing) <= EPS) cycle
        in_dir = v_safe_unit(incoming)
        out_dir = v_safe_unit(outgoing)

        ! The side of the chain the bend currently lies on decides which way the
        ! corrected chain folds. A perfectly straight or perfectly folded joint
        ! has no such side, so pick a stable perpendicular instead - without it a
        ! straight chain could never be given any maximum below 180 degrees.
        side = v_sub(out_dir, v_scale(in_dir, v_dot(out_dir, in_dir)))
        if (v_len2(side) <= EPS) then
          side = v_perp(in_dir)
        else
          side = v_safe_unit(side)
        end if
        desired = v_safe_unit(v_add(v_scale(in_dir, cos(deg_to_rad(clamped))), &
                                   v_scale(side, sin(deg_to_rad(clamped)))))
        correction = q_from_two_vectors(out_dir, desired)
        do j = i + 1, joint_count
          joints(:, j) = v_add(pivot, q_xform(correction, v_sub(joints(:, j), pivot)))
        end do
        projections = projections + 1
        projected = .true.
      end do
      if (.not. projected) exit
    end do

    do i = 2, joint_count - 1
      if (i > limit_count) exit
      if (joint_limits(1, i) >= joint_limits(2, i)) cycle
      minimum = max(0.0_real32, min(180.0_real32, joint_limits(1, i)))
      maximum = max(0.0_real32, min(180.0_real32, joint_limits(2, i)))
      theta = interior_angle_degrees(joints(:, i - 1), joints(:, i), joints(:, i + 1))
      if (theta < minimum - 0.01_real32 .or. theta > maximum + 0.01_real32) then
        violations = violations + 1
      end if
    end do
  end subroutine apply_joint_limits

  pure subroutine derive_rotations(joint_count, joints, rotations)
    ! One orientation per bone. Bone i spans towards joint i+1, and the last joint
    ! keeps the direction of the final segment.
    !
    ! The frame is seeded with a reference axis that is not parallel to the first
    ! bone and then parallel-transported, so bending a chain never flips a bone
    ! the way an independently computed up-vector would.
    integer, intent(in) :: joint_count
    real(real32), intent(in) :: joints(3, joint_count)
    real(real32), intent(out) :: rotations(4, joint_count)
    real(real32) :: directions(3, joint_count), reference(3), side(3), up(3)
    real(real32) :: forward(3), step(4), frame(4), fallback(3)
    integer :: i

    if (joint_count < 2) return
    do i = 1, joint_count - 1
      if (v_len2(v_sub(joints(:, i + 1), joints(:, i))) > EPS) then
        directions(:, i) = v_safe_unit(v_sub(joints(:, i + 1), joints(:, i)))
      else
        directions(:, i) = [0.0_real32, 1.0_real32, 0.0_real32]
      end if
    end do
    directions(:, joint_count) = directions(:, joint_count - 1)

    reference = [0.0_real32, 1.0_real32, 0.0_real32]
    if (abs(directions(1, 1)) > 0.99_real32) then
      reference = [1.0_real32, 0.0_real32, 0.0_real32]
    end if
    side = v_safe_unit(v_cross(reference, directions(:, 1)))
    frame = q_identity()

    do i = 1, joint_count
      up = directions(:, i)
      if (i > 1) then
        step = q_from_two_vectors(directions(:, i - 1), up)
        frame = q_mul(step, frame)
        side = q_xform(step, side)
      end if
      ! Re-orthogonalise against drift accumulated by the transport.
      side = v_sub(side, v_scale(up, v_dot(side, up)))
      if (v_len2(side) <= EPS) then
        fallback = [1.0_real32, 0.0_real32, 0.0_real32]
        if (abs(fallback(1)) > 0.99_real32) then
          fallback = [0.0_real32, 0.0_real32, 1.0_real32]
        end if
        side = v_safe_unit(v_cross(fallback, up))
      end if
      side = v_safe_unit(side)
      forward = v_safe_unit(v_cross(side, up))
      rotations(:, i) = q_from_basis(side, up, forward)
    end do
  end subroutine derive_rotations

  pure subroutine smooth_rotations(joint_count, joints, lengths, previous, rotations, &
      smoothing, out_joints, out_rotations)
    ! Ease towards the solved orientation in ROTATION space, then rebuild the
    ! positions by forward kinematics with the declared lengths.
    !
    ! Interpolating joint POSITIONS towards the solved ones looks right for one
    ! frame and is wrong in a way that matters: it changes every segment length,
    ! so the rig's bones stretch and the visual skeleton stops matching the
    ! declared one. The humanoid demo caught exactly this - a 0.30 m upper arm
    ! rendered as 0.264 m while the chain eased. Slerping the orientations and
    ! rebuilding cannot stretch a bone, and it converges to the raw solve because
    ! slerp(previous, raw, 1.0) IS the raw rotation.
    !
    ! The bone convention is local +Y, the same one pose_skeleton uses.
    integer, intent(in) :: joint_count
    real(real32), intent(in) :: joints(3, joint_count), lengths(joint_count - 1)
    real(real32), intent(in) :: previous(4, joint_count), rotations(4, joint_count)
    real(real32), intent(in) :: smoothing
    real(real32), intent(out) :: out_joints(3, joint_count), out_rotations(4, joint_count)
    real(real32) :: weight, length
    integer :: i

    out_joints = joints
    out_rotations = rotations
    if (smoothing <= 0.0_real32) return
    if (joint_count < 2) return
    if (.not. all_finite([smoothing])) return
    if (.not. all_finite(reshape(previous, [4 * joint_count]))) return
    if (.not. all_finite(reshape(rotations, [4 * joint_count]))) return

    weight = 1.0_real32 - smoothing
    do i = 1, joint_count
      out_rotations(:, i) = q_slerp(previous(:, i), rotations(:, i), weight)
    end do
    do i = 1, joint_count - 1
      length = lengths(i)
      if (.not. all_finite([length])) cycle
      if (length <= EPS) cycle
      out_joints(:, i + 1) = v_add(out_joints(:, i), &
          v_scale(q_xform(out_rotations(:, i), [0.0_real32, 1.0_real32, 0.0_real32]), length))
    end do
  end subroutine smooth_rotations

  pure function blend_influence(current, goal, influence) result(blended)
    ! GodotIK's influence, matched exactly: the leaf's CURRENT position is lerped
    ! towards the effector, measured against the pose each frame rather than a
    ! remembered one. 0 leaves the chain where it is, 1 goes straight to the goal.
    real(real32), intent(in) :: current(3), goal(3), influence
    real(real32) :: blended(3)
    real(real32) :: t
    if (.not. all_finite([influence])) return
    t = max(0.0_real32, min(1.0_real32, influence))
    blended = v_add(v_scale(current, 1.0_real32 - t), v_scale(goal, t))
  end function blend_influence

end module fabrik_pipeline
