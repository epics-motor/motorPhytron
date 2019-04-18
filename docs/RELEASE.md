# motorPhytron Releases

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
