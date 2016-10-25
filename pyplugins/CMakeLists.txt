# Copyright (c) 2016 LAAS-CNRS
# Author: Joseph Mirabel
#
# This file is part of hpp-rbprm-corba.
# hpp-rbprm-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-rbprm-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

FINDPYTHON()
PYTHON_INSTALL_ON_SITE (gepetto/gui/affordance/ affordancewidget.py)
PYTHON_INSTALL_ON_SITE (gepetto/gui/affordance/ __init__.py)
SET (RBPRM_CONFIG
    gepetto/gui/affordance/rbprmEnvironments.conf
    gepetto/gui/affordance/rbprmRoms.conf
    gepetto/gui/affordance/rbprmFullbodies.conf
    )

install (FILES
    ${RBPRM_CONFIG}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/etc/gepetto-gui)


