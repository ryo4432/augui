import math


def transform2d(x, y, theta):
    new_x = math.cos(theta) * x + math.sin(theta) * y
    new_y = - math.sin(theta) * x + math.cos(theta) * y
    return (new_x, new_y)


#  lat-lon <-> x-y conversion: https://scienceweb.whoi.edu/marine/ndsf/utility/NDSFutility.html

# /* coordinate translation options */
XY_TO_LL = 1
LL_TO_XY = 2


def DEG_TO_RADIANS(deg):
    return deg / 180.0 * math.pi


def METERS_DEGLON(x):
    d2r = DEG_TO_RADIANS(x)
    ret = (111415.13 * math.cos(d2r)) - (94.55 * math.cos(3.0 * d2r)) + (0.12 * math.cos(5.0 * d2r))
    return ret


def METERS_DEGLAT(x):
    d2r = DEG_TO_RADIANS(x)
    ret = 111132.09 - (566.05 * math.cos(2.0 * d2r)) + (1.20 * math.cos(4.0 * d2r)) - (0.002 * math.cos(6.0 * d2r))
    return ret


# /*----------------------------------------------------------
#   The following functions are modified from the origin
#   C functions created by Louis Whitcomb 19 Jun 2001
# ---------------------------------------------------------*/
# /*----------------------------------------------------------
#   translate_coordinates
#   routine to translate between geographic and cartesian coordinates
#   user must supply following data on the cartesian coordinate system:
#   location of the origin in lat/lon degrees
#   rotational skew from true north in degrees
#   N.B. sense of rotation i/p here is exactly as o/p by ORIGIN
#   x/y offset in meters - only if an offset was used during the
#   running of prog ORIGIN
# */
def translate_coordinates(trans_option, porg):

    angle = DEG_TO_RADIANS(porg["rotation_angle_degs"])

    if trans_option == XY_TO_LL:
        # /* X,Y to Lat/Lon Coordinate Translation  */
        pxpos_mtrs = porg["x"]
        pypos_mtrs = porg["y"]
        xx = pxpos_mtrs - porg["xoffset_mtrs"]
        yy = pypos_mtrs - porg["yoffset_mtrs"]
        r = math.sqrt(xx * xx + yy * yy)

        if r:
            ct = xx / r
            st = yy / r
            xx = r * ((ct * math.cos(angle)) + (st * math.sin(angle)))
            yy = r * ((st * math.cos(angle)) - (ct * math.sin(angle)))

        plon = porg["olon"] + xx / METERS_DEGLON(porg["olat"])
        plat = porg["olat"] + yy / METERS_DEGLAT(porg["olat"])
        return plat, plon
        
    elif trans_option == LL_TO_XY:
        xx = (porg["slon"] - porg["olon"]) * METERS_DEGLON(porg["olat"])
        yy = (porg["slat"] - porg["olat"]) * METERS_DEGLAT(porg["olat"])

        r = math.sqrt(xx * xx + yy * yy)

        # /* alert('LL_TO_XY: xx=' + xx + ' yy=' + yy + ' r=' + r)
        # return false*/

        if r:
            ct = xx / r
            st = yy / r
            xx = r * ((ct * math.cos(angle)) + (st * math.sin(angle)))
            yy = r * ((st * math.cos(angle)) - (ct * math.sin(angle)))

        pxpos_mtrs = xx + porg["xoffset_mtrs"]
        pypos_mtrs = yy + porg["yoffset_mtrs"]
        return pxpos_mtrs, pypos_mtrs


def xy2ll(olat, olon, sx, sy):
    origin = {
        "x": sx,
        "y": sy,
        "coord_system": 1,
        "olat": olat,
        "olon": olon,
        "xoffset_mtrs": 0,
        "yoffset_mtrs": 0,
        "rotation_angle_degs": 0,
        "rms_error": 0
    }
    return translate_coordinates(XY_TO_LL, origin)
