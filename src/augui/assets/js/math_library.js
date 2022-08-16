function transform2d([x, y], theta) {
   new_x =   Math.cos(theta) * x + Math.sin(theta) * y
   new_y = - Math.sin(theta) * x + Math.cos(theta) * y
   return [new_x, new_y]
}

// lat-lon <-> x-y conversion: https://scienceweb.whoi.edu/marine/ndsf/utility/NDSFutility.html

/* coordinate translation options */
XY_TO_LL= 1;
LL_TO_XY= 2;

function DEG_TO_RADIANS(deg) {
   return deg / 180.0 * Math.PI;
}

function METERS_DEGLON(x)
{
   var d2r=DEG_TO_RADIANS(x);
   return((111415.13 * Math.cos(d2r))- (94.55 * Math.cos(3.0*d2r)) + (0.12 * Math.cos(5.0*d2r)));
}

function METERS_DEGLAT(x)
{
      var d2r=DEG_TO_RADIANS(x);
      return(111132.09 - (566.05 * Math.cos(2.0*d2r))+ (1.20 * Math.cos(4.0*d2r)) - (0.002 * Math.cos(6.0*d2r)));
}

/*----------------------------------------------------------
#   The following functions are modified from the origin
#   C functions created by Louis Whitcomb 19 Jun 2001
---------------------------------------------------------*/
/*----------------------------------------------------------
#   translate_coordinates
#   routine to translate between geographic and cartesian coordinates
#   user must supply following data on the cartesian coordinate system:
#   location of the origin in lat/lon degrees;
#   rotational skew from true north in degrees;
#   N.B. sense of rotation i/p here is exactly as o/p by ORIGIN
#   x/y offset in meters - only if an offset was used during the
#   running of prog ORIGIN;
*/
function translate_coordinates(trans_option, porg)
{
   var xx, yy, r, ct, st, angle;
   angle = DEG_TO_RADIANS(porg.rotation_angle_degs);

   if( trans_option == XY_TO_LL)
   {
      /* X,Y to Lat/Lon Coordinate Translation  */
      pxpos_mtrs = porg.x;  
      pypos_mtrs = porg.y;
      xx = pxpos_mtrs - porg.xoffset_mtrs;
      yy = pypos_mtrs - porg.yoffset_mtrs;
      r = Math.sqrt(xx*xx + yy*yy);

      if(r)
      {
         ct = xx / r;
         st = yy / r;
         xx = r * ( (ct * Math.cos(angle)) + (st * Math.sin(angle)) );
         yy = r * ( (st * Math.cos(angle)) - (ct * Math.sin(angle)) );
      }

      var plon = porg.olon + xx / METERS_DEGLON(porg.olat);
      var plat = porg.olat + yy / METERS_DEGLAT(porg.olat);

      var sll={};
      sll={slat:plat, slon:plon};
      return(sll);
   }
   else if(trans_option == LL_TO_XY)
   {
      xx = (porg.slon - porg.olon) * METERS_DEGLON(porg.olat);
      yy = (porg.slat - porg.olat) * METERS_DEGLAT(porg.olat);

      r = Math.sqrt(xx*xx + yy*yy);

      /* alert('LL_TO_XY: xx=' + xx + ' yy=' + yy + ' r=' + r);
      return false;*/

      if(r)
      {
         ct = xx / r;
         st = yy / r;
         xx = r * ( (ct * Math.cos(angle)) + (st * Math.sin(angle)) );
         yy = r * ( (st * Math.cos(angle)) - (ct * Math.sin(angle)) );
      }
      pxpos_mtrs = xx + porg.xoffset_mtrs;
      pypos_mtrs = yy + porg.yoffset_mtrs;
         
      var sxy={};
      sxy={x:pxpos_mtrs, y:pypos_mtrs};

      return(sxy);
   }
}


function xy2ll([olat, olon], [sx, sy]) {
   var origin = {
      x: sx,
      y: sy,
      coord_system: 1,
      olat: olat,
      olon: olon,
      xoffset_mtrs: 0,
      yoffset_mtrs:0, 
      rotation_angle_degs: 0,
      rms_error: 0
   };
   
   var xy2ll = translate_coordinates(XY_TO_LL, origin); 
   

   /* get the results and fill in the form */
   var slat = xy2ll.slat;  /* in decimal degrees */
   var slon = xy2ll.slon; 
   return [slat, slon];
}