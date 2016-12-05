classdef robot
   properties
      pos
      v %fwd velocity
      w %augular velocity
   end
   methods
      function obj = robot(pp,vv,ww)
         if nargin > 0
            obj.pos = pp;
            obj.v = vv;
            obj.w = ww;
         end
      end
      function 
   end
end