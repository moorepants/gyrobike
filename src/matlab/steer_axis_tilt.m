function lambda = steer_axis_tilt(rF, rR, a, b, c)
% function lambda = steer_axis_tilt(rF, rR, a, b, c)
%
% Returns the steer axis tilt, lambda, for the parameter set based on the
% offsets from the steer axis.
%
% Parameters
% ----------
% rF : double
%   Front wheel radius.
% rR : double
%   Rear wheel radius.
% a : float
%   The rear wheel offset from the steer axis.
% b : float
%   The front wheel offset from the steer axis.
% c : float
%   The distance along the steer axis between the front wheel and rear
%   wheel.
%
% Returns
% -------
% lambda : double
%   The steer axis tilt as described in Meijaard2007.
%

f = @(lam) sin(lam) - (rF - rR + c * cos(lam)) / (a + b);
guess = atan(c / (a + b)); % guess based on equal wheel radii
lambda = fzero(f, guess);
