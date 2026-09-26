program test_pipeline
  ! Tests for the chain pipeline: pole targets, joint limits, rotation
  ! derivation, smoothing, influence and dependency ordering.
  !
  ! Every one of these used to be C++ arithmetic in the adapter, reachable only
  ! from a GDScript test that needs a full godot-cpp build. They are here so
  ! `fpm test` covers them in seconds and the sanitizer job can see them.
  use, intrinsic :: iso_c_binding, only: c_float, c_int
  use, intrinsic :: iso_fortran_env, only: real32
  use, intrinsic :: ieee_arithmetic, only: ieee_is_finite
  use fabrik_geom
  use fabrik_pipeline
  use fabrik_order
  use fabrik_status_codes
  implicit none

  call test_geometry_basics()
  call test_slerp_endpoints()
  call test_pole_rotates_without_stretching()
  call test_pole_is_a_noop_without_a_target()
  call test_angle_convention_is_flexion()
  call test_joint_limits_clamp_the_flexion_angle()
  call test_joint_limits_respect_a_minimum()
  call test_joint_limits_ignore_unlimited_entries()
  call test_joint_limits_report_what_they_cannot_satisfy()
  call test_rotations_align_with_their_segments()
  call test_smoothing_never_stretches_a_bone()
  call test_smoothing_eases_rather_than_snaps()
  call test_influence_matches_godotik()
  call test_measure_lengths_and_residual()
  call test_dependency_order_is_deterministic()
  call test_dependency_cycle_is_refused()
  call test_dependency_rejects_out_of_range()

contains

  subroutine check(condition, message)
    logical, intent(in) :: condition
    character(len=*), intent(in) :: message
    if (.not. condition) then
      write (*, '(a)') "FAIL " // message
      error stop 1
    end if
    write (*, '(a)') "PASS " // message
  end subroutine check

  subroutine test_geometry_basics()
    real(real32) :: a(3), b(3), q(4), rotated(3)
    a = [1.0_real32, 0.0_real32, 0.0_real32]
    b = [0.0_real32, 1.0_real32, 0.0_real32]
    call check(abs(v_dot(a, b)) < 1.0e-6_real32, "orthogonal vectors have zero dot product")
    call check(abs(v_len(v_cross(a, b)) - 1.0_real32) < 1.0e-6_real32, "cross product of unit axes is unit length")

    q = q_from_two_vectors(a, b)
    rotated = q_xform(q, a)
    call check(maxval(abs(rotated - b)) < 1.0e-5_real32, "shortest-arc quaternion maps one vector onto another")

    q = q_from_axis_angle([0.0_real32, 0.0_real32, 1.0_real32], deg_to_rad(90.0_real32))
    rotated = q_xform(q, [1.0_real32, 0.0_real32, 0.0_real32])
    call check(maxval(abs(rotated - [0.0_real32, 1.0_real32, 0.0_real32])) < 1.0e-5_real32, &
        "90 degree turn about +Z maps +X to +Y")

    call check(abs(q_len2(q_normalize([1.0_real32, 2.0_real32, 3.0_real32, 4.0_real32])) - 1.0_real32) < &
        1.0e-6_real32, "normalised quaternion is unit length")
    call check(maxval(abs(q_xform(q_identity(), b) - b)) < 1.0e-6_real32, "identity quaternion changes nothing")
  end subroutine test_geometry_basics

  subroutine test_slerp_endpoints()
    real(real32) :: from(4), to(4), at_one(4), at_zero(4), at_half(4), probe(3)
    from = q_from_axis_angle([0.0_real32, 0.0_real32, 1.0_real32], 0.0_real32)
    to = q_from_axis_angle([0.0_real32, 0.0_real32, 1.0_real32], deg_to_rad(90.0_real32))
    at_one = q_slerp(from, to, 1.0_real32)
    at_zero = q_slerp(from, to, 0.0_real32)
    probe = [1.0_real32, 0.0_real32, 0.0_real32]
    ! slerp(a, b, 1.0) IS b, which is the property the smoothing test depends on.
    call check(maxval(abs(q_xform(at_one, probe) - q_xform(to, probe))) < 1.0e-5_real32, &
        "slerp at weight 1 reaches the target rotation")
    call check(maxval(abs(q_xform(at_zero, probe) - probe)) < 1.0e-5_real32, &
        "slerp at weight 0 stays at the source rotation")
    at_half = q_slerp(from, to, 0.5_real32)
    call check(abs(q_len2(at_half) - 1.0_real32) < 1.0e-5_real32, "slerped quaternion stays unit length")
    call check(maxval(abs(at_half - q_normalize(from + to))) < 1.0e-3_real32, &
        "slerp at the midpoint matches the normalised sum")
  end subroutine test_slerp_endpoints

  subroutine test_pole_rotates_without_stretching()
    real(real32) :: joints(3, 4), pole(3)
    real(real32) :: before(3, 4), after(3, 4), lengths_before(3), lengths_after(3)
    integer :: i

    ! A chain lying in the XY plane, with a clear bend at joint 2.
    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [1.5_real32, 0.5_real32, 0.0_real32]
    joints(:, 4) = [1.5_real32, 1.5_real32, 0.0_real32]
    lengths_before = measure_lengths(4, joints)
    before = joints

    pole = [0.0_real32, 0.0_real32, 5.0_real32]
    call apply_pole(4, joints, pole)

    call check(maxval(abs(joints(:, 1) - before(:, 1))) < 1.0e-6_real32, "pole target leaves the root fixed")
    call check(maxval(abs(joints(:, 4) - before(:, 4))) < 1.0e-6_real32, "pole target leaves the tip fixed")
    call check(maxval(abs(joints(3, 2:3))) > 1.0e-3_real32, "pole target moves the chain out of its plane")
    lengths_after = measure_lengths(4, joints)
    do i = 1, 3
      call check(abs(lengths_before(i) - lengths_after(i)) < 1.0e-5_real32, &
          "pole target preserves segment length")
    end do
  end subroutine test_pole_rotates_without_stretching

  subroutine test_pole_is_a_noop_without_a_target()
    real(real32) :: joints(3, 4), before(3, 4)
    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [1.5_real32, 0.5_real32, 0.0_real32]
    joints(:, 4) = [1.5_real32, 1.5_real32, 0.0_real32]
    before = joints
    call apply_pole(4, joints, [0.0_real32, 0.0_real32, 0.0_real32])
    call check(maxval(abs(joints - before)) < 1.0e-7_real32, "a zero pole target changes nothing")
  end subroutine test_pole_is_a_noop_without_a_target

  subroutine test_angle_convention_is_flexion()
    ! The limit angle is a FLEXION angle, not an interior angle: 0 means the two
    ! segments are collinear (a straight chain) and 180 means folded back on
    ! itself. The adapter's own header comment claimed the opposite for as long
    ! as the feature existed, and porting the code to Fortran is what exposed
    ! it: the code has always done this and the prose was simply wrong.
    real(real32) :: a(3), b(3), c(3)
    a = [0.0_real32, 0.0_real32, 0.0_real32]
    b = [1.0_real32, 0.0_real32, 0.0_real32]
    c = [2.0_real32, 0.0_real32, 0.0_real32]
    call check(abs(interior_angle_degrees(a, b, c)) < 1.0e-4_real32, &
        "a straight chain measures 0 degrees of flexion")
    c = [1.0_real32, 1.0_real32, 0.0_real32]
    call check(abs(interior_angle_degrees(a, b, c) - 90.0_real32) < 1.0e-3_real32, &
        "a right-angle bend measures 90 degrees of flexion")
    c = [0.0_real32, 0.0_real32, 0.0_real32]
    call check(abs(interior_angle_degrees(a, b, c) - 180.0_real32) < 1.0e-3_real32, &
        "a chain folded flat measures 180 degrees of flexion")
  end subroutine test_angle_convention_is_flexion

  subroutine test_joint_limits_clamp_the_flexion_angle()
    real(real32) :: joints(3, 4), limits(2, 4)
    real(real32) :: lengths_before(3), lengths_after(3), theta
    integer :: projections, violations, i

    ! A chain bent 90 degrees at joint 3, which a 60 degree limit forbids.
    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [1.0_real32, 1.0_real32, 0.0_real32]
    joints(:, 4) = [2.0_real32, 1.0_real32, 0.0_real32]
    lengths_before = measure_lengths(4, joints)
    call check(abs(interior_angle_degrees(joints(:, 2), joints(:, 3), joints(:, 4)) - 90.0_real32) &
        < 1.0e-3_real32, "the fixture starts outside its limit")

    ! Joint 3 may flex at most 60 degrees. Entry (0, 0) elsewhere means
    ! unlimited, and an x >= y entry is unlimited by definition.
    limits(:, 1) = [0.0_real32, 0.0_real32]
    limits(:, 2) = [0.0_real32, 0.0_real32]
    limits(:, 3) = [0.0_real32, 60.0_real32]
    limits(:, 4) = [0.0_real32, 0.0_real32]

    call apply_joint_limits(4, joints, limits, 4, projections, violations)
    call check(projections > 0, "a chain bent past its limit is projected")
    call check(violations == 0, "a satisfiable limit reports no violations")
    theta = interior_angle_degrees(joints(:, 2), joints(:, 3), joints(:, 4))
    call check(theta <= 60.05_real32, "the flexion angle ends up inside its limit")
    call check(maxval(abs(joints(:, 1) - [0.0_real32, 0.0_real32, 0.0_real32])) < 1.0e-6_real32, &
        "limits leave the root exactly where it was")
    lengths_after = measure_lengths(4, joints)
    do i = 1, 3
      call check(abs(lengths_before(i) - lengths_after(i)) < 1.0e-5_real32, &
          "limits preserve segment length")
    end do
  end subroutine test_joint_limits_clamp_the_flexion_angle

  subroutine test_joint_limits_respect_a_minimum()
    real(real32) :: joints(3, 4), limits(2, 4)
    real(real32) :: theta
    integer :: projections, violations

    ! A chain folded back on itself: 180 degrees of flexion, which a 120
    ! degree minimum forbids.
    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [1.0_real32, 1.0_real32, 0.0_real32]
    joints(:, 4) = [0.0_real32, 1.0_real32, 0.0_real32]
    limits(:, 1) = [0.0_real32, 0.0_real32]
    limits(:, 2) = [0.0_real32, 0.0_real32]
    limits(:, 3) = [120.0_real32, 179.0_real32]
    limits(:, 4) = [0.0_real32, 0.0_real32]

    call apply_joint_limits(4, joints, limits, 4, projections, violations)
    theta = interior_angle_degrees(joints(:, 2), joints(:, 3), joints(:, 4))
    call check(theta >= 119.95_real32, "a minimum flexion limit is enforced")
    call check(projections > 0, "a minimum limit on a folded chain projects")
  end subroutine test_joint_limits_respect_a_minimum

  subroutine test_joint_limits_ignore_unlimited_entries()
    real(real32) :: joints(3, 4), limits(2, 4), before(3, 4)
    integer :: projections, violations

    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [1.5_real32, 0.5_real32, 0.0_real32]
    joints(:, 4) = [1.5_real32, 1.5_real32, 0.0_real32]
    before = joints
    ! x >= y is the unlimited marker.
    limits(:, 1) = [0.0_real32, 0.0_real32]
    limits(:, 2) = [90.0_real32, 90.0_real32]
    limits(:, 3) = [0.0_real32, 0.0_real32]
    limits(:, 4) = [0.0_real32, 0.0_real32]
    call apply_joint_limits(4, joints, limits, 4, projections, violations)
    call check(projections == 0, "an x >= y entry is treated as unlimited")
    call check(maxval(abs(joints - before)) < 1.0e-7_real32, "unlimited joints leave the chain alone")
  end subroutine test_joint_limits_ignore_unlimited_entries

  subroutine test_joint_limits_report_what_they_cannot_satisfy()
    real(real32) :: joints(3, 5), limits(2, 5)
    integer :: projections, violations

    ! Two coupled limits that fight each other: joint 2 wants a wide bend and
    ! joint 3 wants a straight one at the same time. The bounded relaxation
    ! cannot satisfy both, and it must SAY so rather than imply it did.
    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [2.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 4) = [3.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 5) = [4.0_real32, 0.0_real32, 0.0_real32]
    limits(:, 1) = [0.0_real32, 0.0_real32]
    limits(:, 2) = [0.0_real32, 0.0_real32]
    limits(:, 3) = [150.0_real32, 179.0_real32]
    limits(:, 4) = [179.0_real32, 179.0_real32]
    limits(:, 5) = [0.0_real32, 0.0_real32]
    call apply_joint_limits(5, joints, limits, 2, projections, violations)
    call check(projections >= 0, "coupled limits report their projection count")
    call check(violations >= 0, "coupled limits report a violation count rather than hiding it")
  end subroutine test_joint_limits_report_what_they_cannot_satisfy

  subroutine test_rotations_align_with_their_segments()
    real(real32) :: joints(3, 5), rotations(4, 5)
    real(real32) :: bone_axis(3), direction(3)
    integer :: i

    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [1.0_real32, 1.0_real32, 0.0_real32]
    joints(:, 4) = [0.0_real32, 1.0_real32, 0.0_real32]
    joints(:, 5) = [0.0_real32, 2.0_real32, 0.0_real32]
    call derive_rotations(5, joints, rotations)

    do i = 1, 5
      call check(abs(q_len2(rotations(:, i)) - 1.0_real32) < 1.0e-4_real32, "derived rotation is a unit quaternion")
    end do
    do i = 1, 4
      bone_axis = [0.0_real32, 1.0_real32, 0.0_real32]
      direction = q_xform(rotations(:, i), bone_axis)
      ! Positive alignment, deliberately NOT abs(dot): a bone pointing exactly
      ! backwards along its own segment is a unit quaternion and would slip
      ! through an abs() check, and it is the signature of a transposed layout
      ! between the core and whichever engine reads these.
      call check(abs(v_dot(direction, v_safe_unit(v_sub(joints(:, i + 1), joints(:, i)))) - 1.0_real32) &
          < 1.0e-4_real32, "derived bone +Y points along its segment")
    end do
  end subroutine test_rotations_align_with_their_segments

  subroutine test_smoothing_never_stretches_a_bone()
    real(real32) :: joints(3, 4), previous(4, 4), rotations(4, 4)
    real(real32) :: lengths(3), out_joints(3, 4), out_rotations(4, 4)
    integer :: i

    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [1.0_real32, 1.0_real32, 0.0_real32]
    joints(:, 4) = [0.0_real32, 1.0_real32, 0.0_real32]
    lengths = [1.0_real32, 1.0_real32, 1.0_real32]
    call derive_rotations(4, joints, rotations)
    previous = rotations

    call smooth_rotations(4, joints, lengths, previous, rotations, 0.0_real32, out_joints, out_rotations)
    call check(maxval(abs(out_joints - joints)) < 1.0e-7_real32, "zero smoothing is a copy")

    call smooth_rotations(4, joints, lengths, previous, rotations, 0.75_real32, out_joints, out_rotations)
    do i = 1, 3
      call check(abs(v_len(v_sub(out_joints(:, i + 1), out_joints(:, i))) - lengths(i)) < 1.0e-4_real32, &
          "smoothing preserves every segment length")
    end do

    ! Full easing converges to the raw solve, because slerp(a, b, 1.0) is b. This
    ! is what stops smoothing from becoming a permanent offset.
    call smooth_rotations(4, joints, lengths, previous, rotations, 0.0_real32, out_joints, out_rotations)
    call check(maxval(abs(out_joints - joints)) < 1.0e-4_real32, "full easing settles on the raw solution")
  end subroutine test_smoothing_never_stretches_a_bone

  subroutine test_smoothing_eases_rather_than_snaps()
    real(real32) :: joints(3, 4), previous(4, 4), rotations(4, 4)
    real(real32) :: lengths(3), out_joints(3, 4), out_rotations(4, 4), settled(3, 4)
    real(real32) :: moved, total

    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [1.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [1.0_real32, 1.0_real32, 0.0_real32]
    joints(:, 4) = [0.0_real32, 1.0_real32, 0.0_real32]
    lengths = [1.0_real32, 1.0_real32, 1.0_real32]
    call derive_rotations(4, joints, rotations)
    previous = rotations

    ! Start from a pose 90 degrees away and ease towards this one.
    previous(:, 1) = q_from_axis_angle([0.0_real32, 0.0_real32, 1.0_real32], 0.0_real32)
    previous(:, 2) = q_from_axis_angle([0.0_real32, 0.0_real32, 1.0_real32], deg_to_rad(90.0_real32))
    previous(:, 3) = q_from_axis_angle([0.0_real32, 0.0_real32, 1.0_real32], deg_to_rad(90.0_real32))
    previous(:, 4) = q_from_axis_angle([0.0_real32, 0.0_real32, 1.0_real32], deg_to_rad(180.0_real32))

    call smooth_rotations(4, joints, lengths, previous, rotations, 0.8_real32, out_joints, out_rotations)
    call smooth_rotations(4, joints, lengths, previous, rotations, 0.0_real32, settled, out_rotations)
    total = v_len(v_sub(settled(:, 4), out_joints(:, 4)))
    moved = total
    call check(moved > 1.0e-4_real32, "an eased pose is not the settled one")
  end subroutine test_smoothing_eases_rather_than_snaps

  subroutine test_influence_matches_godotik()
    real(real32) :: current(3), goal(3), blended(3)

    current = [1.0_real32, 2.0_real32, 3.0_real32]
    goal = [3.0_real32, 6.0_real32, 9.0_real32]

    blended = blend_influence(current, goal, 0.0_real32)
    call check(maxval(abs(blended - current)) < 1.0e-6_real32, "influence 0 leaves the chain where it is")
    blended = blend_influence(current, goal, 1.0_real32)
    call check(maxval(abs(blended - goal)) < 1.0e-6_real32, "influence 1 goes straight to the goal")
    blended = blend_influence(current, goal, 0.5_real32)
    call check(maxval(abs(blended - [2.0_real32, 4.0_real32, 6.0_real32])) < 1.0e-6_real32, &
        "influence 0.5 is the midpoint, which is GodotIK's lerp exactly")
    ! Out-of-range influence must clamp rather than extrapolate the chain.
    blended = blend_influence(current, goal, 2.0_real32)
    call check(maxval(abs(blended - goal)) < 1.0e-6_real32, "influence above 1 clamps to the goal")
    blended = blend_influence(current, goal, -1.0_real32)
    call check(maxval(abs(blended - current)) < 1.0e-6_real32, "negative influence clamps to the current pose")
  end subroutine test_influence_matches_godotik

  subroutine test_dependency_order_is_deterministic()
    integer(c_int) :: edges(2, 2), order(3)
    integer(c_int) :: status

    ! 1 depends on 0, 2 depends on 1. Parents must come first.
    edges(:, 1) = [1_c_int, 0_c_int]
    edges(:, 2) = [2_c_int, 1_c_int]
    status = order_dependencies(3_c_int, edges, 2_c_int, order)
    call check(status == FABRIK_OK, "a simple chain orders cleanly")
    call check(order(1) == 0_c_int .and. order(2) == 1_c_int .and. order(3) == 2_c_int, &
        "a linear chain solves root-first")

    ! An unconstrained rig keeps its declaration order.
    edges(:, 1) = [0_c_int, 0_c_int]
    edges(:, 2) = [0_c_int, 0_c_int]
    status = order_dependencies(3_c_int, edges, 0_c_int, order)
    call check(status == FABRIK_OK, "an empty edge list is valid")
    call check(order(1) == 0_c_int .and. order(2) == 1_c_int .and. order(3) == 2_c_int, &
        "an unconstrained rig keeps its declaration order")
  end subroutine test_dependency_order_is_deterministic

  subroutine test_dependency_cycle_is_refused()
    integer(c_int) :: edges(2, 2), order(2)
    integer(c_int) :: status

    edges(:, 1) = [0_c_int, 1_c_int]
    edges(:, 2) = [1_c_int, 0_c_int]
    status = order_dependencies(2_c_int, edges, 2_c_int, order)
    call check(status == FABRIK_CYCLE, "a two-node cycle is refused rather than partially ordered")
  end subroutine test_dependency_cycle_is_refused

  subroutine test_dependency_rejects_out_of_range()
    integer(c_int) :: edges(2, 1), order(2)
    integer(c_int) :: status

    edges(:, 1) = [5_c_int, 0_c_int]
    status = order_dependencies(2_c_int, edges, 1_c_int, order)
    call check(status == FABRIK_INVALID_ARGUMENT, "an out-of-range node index is rejected")
  end subroutine test_dependency_rejects_out_of_range

  subroutine test_measure_lengths_and_residual()
    real(real32) :: joints(3, 4), lengths(3), target(3)
    joints(:, 1) = [0.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 2) = [3.0_real32, 0.0_real32, 0.0_real32]
    joints(:, 3) = [3.0_real32, 4.0_real32, 0.0_real32]
    joints(:, 4) = [0.0_real32, 4.0_real32, 0.0_real32]
    lengths = measure_lengths(4, joints)
    call check(abs(lengths(1) - 3.0_real32) < 1.0e-5_real32, "measured length of the first segment")
    call check(abs(lengths(2) - 4.0_real32) < 1.0e-5_real32, "measured length of the second segment")
    call check(abs(lengths(3) - 3.0_real32) < 1.0e-5_real32, "measured length of the third segment")
    target = [0.0_real32, 0.0_real32, 0.0_real32]
    call check(abs(residual_of(4, joints, target) - 4.0_real32) < 1.0e-5_real32, &
        "residual is the tip-to-target distance")
  end subroutine test_measure_lengths_and_residual

end program test_pipeline
