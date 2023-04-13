# motorPhytron Releases

## __R1-2 (2023-04-13)__
R1-2 is a release based on the master branch.

### Changes since R1-1

#### New features
* Pull request [#6](https://github.com/epics-motor/motorPhytron/pull/6): [Hasan SANSAR](https://github.com/osiloscop) updated Phytron_I1AM01.db for BiSS interface

#### Modifications to existing features
* None

#### Bug fixes
* Pull request [#3](https://github.com/epics-motor/motorPhytron/pull/3): Fix reference on limit switch from [Bernhard Kuner](https://github.com/bkuner)
* Pull request [#4](https://github.com/epics-motor/motorPhytron/pull/4): [Bernhard Kuner](https://github.com/bkuner) fixed a deceleration bug

#### Continuous integration
* Added ci-scripts (v3.0.1)
* Configured to use Github Actions for CI

## __R1-1 (2020-05-11)__
R1-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* None

#### Modifications to existing features
* Pull request [#1](https://github.com/epics-motor/motorPhytron/pull/1): Only print error messages on status changes

#### Bug fixes
* Commit [72281de](https://github.com/epics-motor/motorPhytron/commit/72281de83391d838f3d0709cb0205a4fa055c426): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorPhytron is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorPhytron can be built outside of the motor directory
* motorPhytron has a dedicated example IOC that can be built outside of motorPhytron

#### Modifications to existing features
* Commit [e2cb686](https://github.com/epics-motor/motorPhytron/commit/e2cb6863910dbc66539ea5ed669b108fafcca72e) phytron.dbd was renamed phytronSupport.dbd to avoid conflicts with the example IOC

#### Bug fixes
* None
