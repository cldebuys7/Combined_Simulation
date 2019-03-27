# matlab_utilities

## Purpose

This package serves as an aggregate of *general* Matlab scripts to be easily accessible elsewhere. This is a ROS package, but the files within it are not all meant to explicitly depend on ROS nor environment variables. From my experience [eac], using a maze of environment variables and random package paths will yield useless complexity and complication, wasted time, and despair.

If you wish your Matlab simulation to be robust, I would strongly suggest using [`git subtree`](https://github.com/git/git/blob/master/contrib/subtree/git-subtree.txt). Specifically, use `git subtree split` to export the `matlab` git directory history, and `git subtree merge` to splice it into your project. Alternatively, you could use `git submodule` to add the subtree as a repository.

**If you are not properly using version control, you are setting yourself and collaborators up for frustration and failure when the unexpected happens.**

## Usage

To use the dependencies in here, simply make sure that the `matlab` direcotry is in your Matlab `path()`, figure out which dependencies you want (i.e., `general`, `sim`, `yaml`, etc.), and execute something like:

	matlab_utilities_depends('general', 'sim', 'yaml', 'ros');

## Using as ROS Package

If you are incorporating `matlab_utilities` as a bonafide ROS package, do the following to add it to your `path()` (this example loads `general` and `yaml`):

	% Load dependencies
	[~, pkg] = system('rospack find matlab_utilities');
	pkg = strtrim(pkg);
	addpath(fullfile(pkg, 'matlab'));
	matlab_utilities_depends('yaml', 'ros');
