__constant sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;
float4 pl_interpolate_cubic4(float X, float4 A, float4 B, float4 C, float4 D) {
    return B + 0.5f * X * (C - A + X * (2.f * A - 5.f * B + 4.f * C - D + X * (3.f * (B - C) + D - A)));
}
__kernel void image_process(__read_only image2d_t src_img,
__write_only image2d_t dst_img ,__global float * gauss,int KERNEL_SIZE, float SIGMA,float detail_alpha,float lowerThreshold,float upperThreshold,float k1,float k2,float k3)
{ 
    int2 image_dim = get_image_dim(src_img);
    int2 pixel_coords = (int2)(get_global_id(0), get_global_id(1)); 
    if (pixel_coords.x >= image_dim.x || pixel_coords.y >= image_dim.y) 
    {return;} 
    float sum = 0.0; 
    float weight_sum = 0.0; 
    for (int i = -KERNEL_SIZE; i <= KERNEL_SIZE; i++) 
    { 
        for (int j = -KERNEL_SIZE; j <= KERNEL_SIZE; j++) 
        { 
            int2 offset = (int2)(j, i); 
            int2 neighbor_coords = pixel_coords + offset; 
            if (neighbor_coords.x >= 0 && neighbor_coords.x < image_dim.x && neighbor_coords.y >= 0 && neighbor_coords.y < image_dim.y) 
            { 
                float weight = gauss[(i+KERNEL_SIZE)*(2*KERNEL_SIZE+1)+j+KERNEL_SIZE];
                float4 pixel = read_imagef(src_img, sampler, neighbor_coords); 
                sum += pixel.x * weight;
                weight_sum += weight; 
            } 
        } 
    }
    
    float4 pixel = read_imagef(src_img, sampler, pixel_coords);
    float detail_0_255 =(pixel.x-sum / weight_sum)*255;
    if (detail_0_255 > -lowerThreshold && detail_0_255 < lowerThreshold) {
           detail_0_255 = detail_0_255 * k1;
        } else if (detail_0_255 >= lowerThreshold && detail_0_255 <= upperThreshold) {
           detail_0_255 = lowerThreshold * k1 + (detail_0_255 - lowerThreshold) * k2;
        } else if (detail_0_255 > upperThreshold) {
            detail_0_255 = lowerThreshold * k1 + (upperThreshold - lowerThreshold) * k2 + (detail_0_255 - upperThreshold) * k3;
        } else if (detail_0_255 <= -lowerThreshold && detail_0_255 >= -upperThreshold) {
            detail_0_255 = -lowerThreshold * k1 + (detail_0_255 + lowerThreshold) * k2;
        } else if (detail_0_255 < -upperThreshold) {
            detail_0_255 = -lowerThreshold * k1 + (-upperThreshold + lowerThreshold) * k2 + (detail_0_255 + upperThreshold) * k3;
        }

    float detail_0_1=detail_0_255 / 255;



    float pixelout = pixel.x+detail_alpha*pixel.x*detail_0_1;
    write_imagef(dst_img, pixel_coords, pixelout); 
}

__kernel void interpolate_image(__read_only image2d_t  image_in,
                                      __write_only image2d_t image_out,float Ratio
                                      ) {
    int i = get_global_id(0);
    int j = get_global_id(1);
    float2 coords = (float2)((float)(i)/Ratio,(float)(j)/Ratio); 
    int rowA = floor(coords.y) - 1;
    int rowB = rowA + 1;
    int rowC = rowB + 1;
    int rowD = rowC + 1;
    
    int colA = floor(coords.x) - 1;
    int colB = colA + 1;
    int colC = colB + 1;
    int colD = colC + 1;
    
    float4 pixelAA = read_imagef(image_in, sampler, (int2)(colA, rowA));
    float4 pixelAB = read_imagef(image_in, sampler, (int2)(colB, rowA));
    float4 pixelAC = read_imagef(image_in, sampler, (int2)(colC, rowA));
    float4 pixelAD = read_imagef(image_in, sampler, (int2)(colD, rowA));
    
    float4 pixelBA = read_imagef(image_in, sampler, (int2)(colA, rowB));
    float4 pixelBB = read_imagef(image_in, sampler, (int2)(colB, rowB));
    float4 pixelBC = read_imagef(image_in, sampler, (int2)(colC, rowB));
    float4 pixelBD = read_imagef(image_in, sampler, (int2)(colD, rowB));
    
    float4 pixelCA = read_imagef(image_in, sampler, (int2)(colA, rowC));
    float4 pixelCB = read_imagef(image_in, sampler, (int2)(colB, rowC));
    float4 pixelCC = read_imagef(image_in, sampler, (int2)(colC, rowC));
    float4 pixelCD = read_imagef(image_in, sampler, (int2)(colD, rowC));
    
    float4 pixelDA = read_imagef(image_in, sampler, (int2)(colA, rowD));
    float4 pixelDB = read_imagef(image_in, sampler, (int2)(colB, rowD));
    float4 pixelDC = read_imagef(image_in, sampler, (int2)(colC, rowD));
    float4 pixelDD = read_imagef(image_in, sampler, (int2)(colD, rowD));
    
    float2 pos = (float2)(coords.x - colB, coords.y - rowB);
    
    float4 pixel = clamp(pl_interpolate_cubic4(pos.x, 
                                               pl_interpolate_cubic4(pos.y, pixelAA, pixelBA, pixelCA, pixelDA),
                                               pl_interpolate_cubic4(pos.y, pixelBA, pixelBB, pixelCB, pixelDB),
                                               pl_interpolate_cubic4(pos.y, pixelAC, pixelBC, pixelCC, pixelDC),
                                               pl_interpolate_cubic4(pos.y, pixelAD, pixelBD, pixelCD, pixelDD)), (float4)(0.f), (float4)(255.f));
    write_imagef(image_out, (int2)(i, j), pixel);
}