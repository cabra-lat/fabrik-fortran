module fabrik_geom
  ! Vector and quaternion primitives for the chain pipeline.
  !
  ! This module exists so that no arithmetic lives in the C++ adapter. It has no
  ! dependency on any engine type: quaternions are plain real arrays in (w, x, y,
  ! z) order, which is also the order the adapter's Godot Quaternion stores them,
  ! so crossing the boundary is a memcpy in both directions.
  use, intrinsic :: iso_c_binding, only: c_float
  use, intrinsic :: iso_fortran_env, only: real32
  implicit none
  private

  public :: v_add, v_sub, v_scale, v_dot, v_cross, v_len, v_len2, v_norm
  public :: v_safe_unit, v_perp, v_rotate_axis, v_from_two
  public :: q_identity, q_mul, q_xform, q_slerp, q_from_axis_angle
  public :: q_from_two_vectors, q_from_basis, q_normalize, q_len2
  public :: interior_angle_degrees, deg_to_rad, rad_to_deg
  public :: angle_about_axis
  public :: EPS
  public :: all_finite

  real(real32), parameter :: EPS = 1.0e-7_real32

contains

  ! A PURE finiteness test, because ieee_is_finite is declared impure by the
  ! standard and every routine in the pipeline that screens its inputs is pure.
  ! `x - x` is 0 for every finite number and NaN for both NaN and Inf, so this
  ! catches all three. CAVEAT: this is a floating-point identity, so it silently
  ! stops detecting anything if the library is ever built with -ffast-math or
  ! -ffinite-math-only. The C ABI does NOT rely on it: fabrik_pipeline_c_api
  ! screens with ieee_is_finite, which fast-math cannot weaken, so a bad build
  ! fails at the boundary rather than returning garbage.
  pure function all_finite(values) result(ok)
    real(real32), intent(in) :: values(:)
    logical :: ok
    ok = all((values - values) == 0.0_real32)
  end function all_finite

  pure function deg_to_rad(d) result(r)
    real(real32), intent(in) :: d
    real(real32) :: r
    r = d * 0.017453292519943295_real32
  end function deg_to_rad

  pure function rad_to_deg(r) result(d)
    real(real32), intent(in) :: r
    real(real32) :: d
    d = r * 57.29577951308232_real32
  end function rad_to_deg

  pure function v_add(a, b) result(r)
    real(real32), intent(in) :: a(3), b(3)
    real(real32) :: r(3)
    r = a + b
  end function v_add

  pure function v_sub(a, b) result(r)
    real(real32), intent(in) :: a(3), b(3)
    real(real32) :: r(3)
    r = a - b
  end function v_sub

  pure function v_scale(a, s) result(r)
    real(real32), intent(in) :: a(3), s
    real(real32) :: r(3)
    r = a * s
  end function v_scale

  pure function v_dot(a, b) result(r)
    real(real32), intent(in) :: a(3), b(3)
    real(real32) :: r
    r = sum(a * b)
  end function v_dot

  pure function v_cross(a, b) result(r)
    real(real32), intent(in) :: a(3), b(3)
    real(real32) :: r(3)
    r = [a(2) * b(3) - a(3) * b(2), &
         a(3) * b(1) - a(1) * b(3), &
         a(1) * b(2) - a(2) * b(1)]
  end function v_cross

  pure function v_len2(a) result(r)
    real(real32), intent(in) :: a(3)
    real(real32) :: r
    r = sum(a * a)
  end function v_len2

  pure function v_len(a) result(r)
    real(real32), intent(in) :: a(3)
    real(real32) :: r
    r = sqrt(sum(a * a))
  end function v_len

  ! Normalise, falling back to +Z for a zero-length input rather than producing
  ! NaN. Every caller in the pipeline treats a zero-length vector as "no
  ! information" and skips the constraint, so the fallback value never reaches an
  ! output; it exists so that no arithmetic here can manufacture a NaN.
  pure function v_safe_unit(a) result(r)
    real(real32), intent(in) :: a(3)
    real(real32) :: r(3)
    real(real32) :: magnitude
    magnitude = sqrt(sum(a * a))
    if (magnitude > EPS) then
      r = a / magnitude
    else
      r = [0.0_real32, 0.0_real32, 1.0_real32]
    end if
  end function v_safe_unit

  pure function v_norm(a) result(r)
    real(real32), intent(in) :: a(3)
    real(real32) :: r(3)
    r = v_safe_unit(a)
  end function v_norm

  ! A unit vector perpendicular to `a`, chosen by crossing with the least
  ! aligned basis axis so the result is as long as possible. A pure function of
  ! the direction, so two runs of the same input bend the same way.
  pure function v_perp(a) result(r)
    real(real32), intent(in) :: a(3)
    real(real32) :: r(3), basis(3)
    if (abs(a(1)) <= abs(a(2)) .and. abs(a(1)) <= abs(a(3))) then
      basis = [1.0_real32, 0.0_real32, 0.0_real32]
    else if (abs(a(2)) <= abs(a(3))) then
      basis = [0.0_real32, 1.0_real32, 0.0_real32]
    else
      basis = [0.0_real32, 0.0_real32, 1.0_real32]
    end if
    r = v_safe_unit(v_cross(basis, a))
  end function v_perp

  ! Rodrigues rotation of `point` about a unit `axis` by `radians`.
  pure function v_rotate_axis(point, axis, radians) result(r)
    real(real32), intent(in) :: point(3), axis(3), radians
    real(real32) :: r(3), cosine, sine
    cosine = cos(radians)
    sine = sin(radians)
    r = point * cosine + v_cross(axis, point) * sine + axis * (v_dot(axis, point) * (1.0_real32 - cosine))
  end function v_rotate_axis

  pure function v_from_two(a, b) result(q)
    ! Shortest-arc quaternion taking unit vector `a` onto unit vector `b`.
    real(real32), intent(in) :: a(3), b(3)
    real(real32) :: q(4)
    real(real32) :: axis(3), cosine
    cosine = max(-1.0_real32, min(1.0_real32, v_dot(a, b)))
    if (cosine > 1.0_real32 - 1.0e-9_real32) then
      q = [1.0_real32, 0.0_real32, 0.0_real32, 0.0_real32]
      return
    end if
    axis = v_cross(a, b)
    q(1) = 1.0_real32 + cosine
    q(2:4) = axis
    q = q_normalize(q)
  end function v_from_two

  pure function q_identity() result(q)
    real(real32) :: q(4)
    q = [1.0_real32, 0.0_real32, 0.0_real32, 0.0_real32]
  end function q_identity

  pure function q_len2(q) result(r)
    real(real32), intent(in) :: q(4)
    real(real32) :: r
    r = sum(q * q)
  end function q_len2

  pure function q_normalize(q) result(r)
    real(real32), intent(in) :: q(4)
    real(real32) :: r(4), magnitude
    magnitude = sqrt(sum(q * q))
    if (magnitude > EPS) then
      r = q / magnitude
    else
      r = q_identity()
    end if
  end function q_normalize

  pure function q_mul(a, b) result(r)
    ! Hamilton product. The order matters: (a * b) applies b first, which is the
    ! convention the adapter's frame transport relies on.
    real(real32), intent(in) :: a(4), b(4)
    real(real32) :: r(4)
    r(1) = a(1) * b(1) - a(2) * b(2) - a(3) * b(3) - a(4) * b(4)
    r(2) = a(1) * b(2) + a(2) * b(1) + a(3) * b(4) - a(4) * b(3)
    r(3) = a(1) * b(3) - a(2) * b(4) + a(3) * b(1) + a(4) * b(2)
    r(4) = a(1) * b(4) + a(2) * b(3) - a(3) * b(2) + a(4) * b(1)
  end function q_mul

  pure function q_xform(q, v) result(r)
    real(real32), intent(in) :: q(4), v(3)
    real(real32) :: r(3)
    ! v + 2 * w * (u x v) + 2 * u x (u x v), written out to avoid temporaries.
    r = v + 2.0_real32 * q(1) * v_cross(q(2:4), v) + &
        2.0_real32 * v_cross(q(2:4), v_cross(q(2:4), v))
  end function q_xform

  pure function q_from_axis_angle(axis, radians) result(q)
    real(real32), intent(in) :: axis(3), radians
    real(real32) :: q(4), half
    half = radians * 0.5_real32
    q(1) = cos(half)
    q(2:4) = v_safe_unit(axis) * sin(half)
  end function q_from_axis_angle

  pure function q_from_two_vectors(a, b) result(q)
    real(real32), intent(in) :: a(3), b(3)
    real(real32) :: q(4)
    q = v_from_two(v_safe_unit(a), v_safe_unit(b))
  end function q_from_two_vectors

  pure function q_from_basis(x_axis, y_axis, z_axis) result(q)
    ! Rotation quaternion for the orthonormal basis whose COLUMNS are the three
    ! given axes. Shepperd's method: pivot on the largest diagonal term so the
    ! square root is never taken of a near-zero number.
    real(real32), intent(in) :: x_axis(3), y_axis(3), z_axis(3)
    real(real32) :: q(4)
    real(real32) :: m(3, 3), trace_value, s
    m(:, 1) = x_axis
    m(:, 2) = y_axis
    m(:, 3) = z_axis
    trace_value = m(1, 1) + m(2, 2) + m(3, 3)
    if (trace_value > 0.0_real32) then
      s = sqrt(trace_value + 1.0_real32) * 2.0_real32
      q(1) = 0.25_real32 * s
      q(2) = (m(3, 2) - m(2, 3)) / s
      q(3) = (m(1, 3) - m(3, 1)) / s
      q(4) = (m(2, 1) - m(1, 2)) / s
    else if (m(1, 1) > m(2, 2) .and. m(1, 1) > m(3, 3)) then
      s = sqrt(1.0_real32 + m(1, 1) - m(2, 2) - m(3, 3)) * 2.0_real32
      q(1) = (m(3, 2) - m(2, 3)) / s
      q(2) = 0.25_real32 * s
      q(3) = (m(1, 2) + m(2, 1)) / s
      q(4) = (m(1, 3) + m(3, 1)) / s
    else if (m(2, 2) > m(3, 3)) then
      s = sqrt(1.0_real32 + m(2, 2) - m(1, 1) - m(3, 3)) * 2.0_real32
      q(1) = (m(1, 3) - m(3, 1)) / s
      q(2) = (m(1, 2) + m(2, 1)) / s
      q(3) = 0.25_real32 * s
      q(4) = (m(2, 3) + m(3, 2)) / s
    else
      s = sqrt(1.0_real32 + m(3, 3) - m(1, 1) - m(2, 2)) * 2.0_real32
      q(1) = (m(2, 1) - m(1, 2)) / s
      q(2) = (m(1, 3) + m(3, 1)) / s
      q(3) = (m(2, 3) + m(3, 2)) / s
      q(4) = 0.25_real32 * s
    end if
    q = q_normalize(q)
  end function q_from_basis

  pure function q_slerp(from, to, weight) result(r)
    ! Shortest-arc spherical interpolation. Below 0.9995 cosine the two are close
    ! enough that the sine denominator loses precision, so a normalised lerp is
    ! used instead - the difference is below float noise there.
    real(real32), intent(in) :: from(4), to(4), weight
    real(real32) :: r(4), target(4), cosine, sine, omega, scale_from, scale_to
    target = to
    cosine = sum(from * target)
    if (cosine < 0.0_real32) then
      target = -target
      cosine = -cosine
    end if
    if (cosine > 0.9995_real32) then
      r = q_normalize(from * (1.0_real32 - weight) + target * weight)
      return
    end if
    omega = acos(max(-1.0_real32, min(1.0_real32, cosine)))
    sine = sin(omega)
    scale_from = sin((1.0_real32 - weight) * omega) / sine
    scale_to = sin(weight * omega) / sine
    r = from * scale_from + target * scale_to
  end function q_slerp

  pure function angle_about_axis(axis, from, to) result(angle)
    ! Signed angle from unit vector `from` to unit vector `to`, measured around
    ! `axis`. atan2 keeps the sign, which matters for a pole target behind the
    ! chain: an unsigned shortest-angle would rotate the wrong way.
    real(real32), intent(in) :: axis(3), from(3), to(3)
    real(real32) :: angle
    angle = atan2(v_dot(axis, v_cross(from, to)), v_dot(from, to))
  end function angle_about_axis

  pure function interior_angle_degrees(a, b, c) result(degrees)
    ! Angle at b between the incoming segment b-a and the outgoing segment c-b.
    real(real32), intent(in) :: a(3), b(3), c(3)
    real(real32) :: degrees
    real(real32) :: incoming(3), outgoing(3), cosine
    incoming = v_sub(b, a)
    outgoing = v_sub(c, b)
    if (v_len2(incoming) <= EPS .or. v_len2(outgoing) <= EPS) then
      degrees = 0.0_real32
      return
    end if
    cosine = max(-1.0_real32, min(1.0_real32, v_dot(v_safe_unit(incoming), v_safe_unit(outgoing))))
    degrees = rad_to_deg(acos(cosine))
  end function interior_angle_degrees

end module fabrik_geom
