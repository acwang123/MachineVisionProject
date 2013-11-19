function [im] = BlendImages(im1, im2, mask)


[pyrAR,pindAR] = buildLpyr(im1(:,:,1), 4);
[pyrAG,pindAG] = buildLpyr(im1(:,:,2), 4);
[pyrAB,pindAB] = buildLpyr(im1(:,:,3), 4);
[pyrBR,pindBR] = buildLpyr(im2(:,:,1), 4);
[pyrBG,pindBG] = buildLpyr(im2(:,:,2), 4);
[pyrBB,pindBB] = buildLpyr(im2(:,:,3), 4);
[pyrG,pindG] = buildGpyr(mask, 4, fspecial('gaussian', [1 16], 4));
pyrLR = pyrG .* pyrAR + (1 - pyrG) .* pyrBR;
pyrLG = pyrG .* pyrAG + (1 - pyrG) .* pyrBG;
pyrLB = pyrG .* pyrAB + (1 - pyrG) .* pyrBB;
imR = reconLpyr(pyrLR, pindG);
imG = reconLpyr(pyrLG, pindG);
imB = reconLpyr(pyrLB, pindG);
im1(:,:,1) = imR;
im1(:,:,2) = imG;
im1(:,:,3) = imB;
im = im1;


end