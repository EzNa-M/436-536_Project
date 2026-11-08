Hey John

I don't really know how Github works, so forgive the messiness that will ensue.

1. This statespace takes the motor voltage as the input. Its max is set to 6 volts, even though that is the nominal voltage. It also enforces the track length.
2. The motor dynamics are integrated by removing the inductor voltage (L and derivative of armature current). The reasoning from Dr. Chagdes and AI is it's small. I don't think so, but whatever.
  Its a problem for future us. If you include it there is a complicated dependence on the jerk (third derivative) of x and theta. There's some other dependencies on the first and second derivatives of
  theta. Conclusion: keep it small. If we start from a stable equilibrium, straight down, this might be a problem. As it is, since we linearize about our desired position, straight up,
  it shouldn't matter.
  (Starting from the down position, in addition to including this behavior, also requires some new techniques, that we'll have to learn)

3. The simulation includes friction on the pendulum joint. Change bp (currently set to zero). Not sure how we get bp in the first place.

4. I didn't test it, my computer is really slow with MATLAB. Not sure what's up.
