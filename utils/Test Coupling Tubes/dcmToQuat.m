function q = dcmToQuat(dmc)
%{
dcmToQuat - This function convert direction cosine matrix (dcm) to
            quaternion.

INPUTS:
        - dcm, double [3,3], direction cosine matrix.

OUTPUTS:
        - q, double [4,1], quaternion.

CALLED FUNCTIONS: -

REVISIONS:
-
%}