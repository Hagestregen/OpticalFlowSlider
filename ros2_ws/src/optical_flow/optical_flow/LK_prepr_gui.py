#!/usr/bin/env python3
"""
Adaptive Lucas-Kanade Optical Flow Demo for Sequence of Images

Computes optical flow between consecutive pairs of images with optional preprocessing.
Now includes flow computation on original grayscale images for comparison.
Enhanced with edge sharpening for better corner detection.
"""
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

# ==========================
# CONFIGURATION (edit here)
# ==========================
image_paths = [
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage1.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage2.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage3.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage4.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage5.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage6.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage7.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage8.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage9.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage10.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage11.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage12.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage13.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage14.png',
    '/home/docker/OpticalFlowSlider/ros2_ws/src/optical_flow/optical_flow/images/MountedLightImage15.png'
]
out_dir       = 'flow_demo_output2'

# Preprocessing toggles
apply_CLAHE    = True
clip_min       = 1.0
clip_max       = 10.0  # Increased for stronger contrast
c_min          = 5.0
c_max          = 50.0
tile_grid      = (4, 4)  # Smaller grid for more localized enhancement
apply_gaussian = False
apply_gamma    = False
gamma_val      = 1.2
apply_retinex  = False

debug          = False  # Print feature count and save histogram
# ==========================

# Preprocessing functions
def apply_clahe(gray, clip, tile_grid):
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=tile_grid)
    return clahe.apply(gray)

def bilateral_filter(image):
    return cv2.bilateralFilter(image, d=9, sigmaColor=75, sigmaSpace=75)

def sharpen_image(image):
    sharpening_kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
    return cv2.filter2D(image, -1, sharpening_kernel)

def apply_gamma(gray, gamma):
    inv = 1.0 / gamma
    table = (np.arange(256) / 255.0) ** inv * 255
    return cv2.LUT(gray.astype('uint8'), table.astype('uint8'))

def apply_retinex(img):
    # Placeholder: return original
    return img

# ==========================
# Main
# ==========================
def main():
    # Ensure output dir
    os.makedirs(out_dir, exist_ok=True)

    # Load images and convert to grayscale
    images = [cv2.imread(path) for path in image_paths]
    grays = [cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in images]

    # Process each consecutive pair
    for i in range(len(grays) - 1):
        gray1, gray2 = grays[i], grays[i+1]

        # Apply bilateral filter to reduce noise
        filtered1 = bilateral_filter(gray1)
        filtered2 = bilateral_filter(gray2)

        # Apply sharpening to enhance edges
        sharpened1 = sharpen_image(filtered1)
        sharpened2 = sharpen_image(filtered2)

        # --- Flow on original grayscale (with preprocessing) ---
        feature_params = dict(maxCorners=500, qualityLevel=0.1,
                              minDistance=10, blockSize=7)
        lk_params = dict(winSize=(31,31), maxLevel=3,
                         criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                                   50, 0.001))
        p0_orig = cv2.goodFeaturesToTrack(sharpened1, mask=None, **feature_params)
        if p0_orig is None or len(p0_orig) == 0:
            print(f"No features detected in original image {i+1}.")
        else:
            p0_orig = cv2.cornerSubPix(sharpened1, p0_orig, (10,10), (-1,-1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                                        10, 0.03))
            p1_orig, st_orig, _ = cv2.calcOpticalFlowPyrLK(sharpened1, sharpened2, p0_orig, None, **lk_params)
            good0_orig = p0_orig[st_orig.flatten() == 1]
            good1_orig = p1_orig[st_orig.flatten() == 1]
            disp_orig = good1_orig - good0_orig
            if len(disp_orig) > 0:
                disp_reshaped_orig = disp_orig.reshape(-1, 2)
                vx_orig, vy_orig = np.mean(disp_reshaped_orig, axis=0)
                mag_orig = np.sqrt(vx_orig * vx_orig + vy_orig * vy_orig)
                print(f"Original: Mean flow velocity between image {i+1} and {i+2}: vx={vx_orig:.2f}, vy={vy_orig:.2f}, mag={mag_orig:.2f} px/frame")
                # Draw arrows for original
                vis_orig = cv2.cvtColor(sharpened2, cv2.COLOR_GRAY2BGR)
                for (x0,y0),(x1,y1) in zip(good0_orig.reshape(-1,2), good1_orig.reshape(-1,2)):
                    cv2.arrowedLine(vis_orig, (int(x0),int(y0)), (int(x1),int(y1)),
                                    (0,255,0), 1, tipLength=0.3)
                    cv2.circle(vis_orig, (int(x0),int(y0)), 3, (0,0,255), -1)
                cv2.imwrite(os.path.join(out_dir, f'flow_arrows_orig_{i+1}_{i+2}.png'), vis_orig)
            else:
                print(f"Original: No good points found for optical flow between image {i+1} and {i+2}.")

        # Preprocessing for both images
        pre1, pre2 = gray1.copy(), gray2.copy()

        if apply_CLAHE:
            def adaptive_clip(g):
                contrast = np.std(g) / (np.mean(g) + 1e-3)
                clip = np.clip(
                    clip_min + (contrast - c_min)/(c_max - c_min)*(clip_max - clip_min),
                    clip_min, clip_max)
                print(f"Image {i+1}: Contrast = {contrast:.2f}, Clip Limit = {clip:.2f}")
                return clip
            pre1 = apply_clahe(pre1, adaptive_clip(pre1), tile_grid)
            pre2 = apply_clahe(pre2, adaptive_clip(pre2), tile_grid)
            pre1 = sharpen_image(pre1)  # Apply enhanced sharpening
            pre2 = sharpen_image(pre2)

        if apply_retinex:
            img1_r = apply_retinex(images[i])
            img2_r = apply_retinex(images[i+1])
            pre1 = cv2.cvtColor(img1_r, cv2.COLOR_BGR2GRAY)
            pre2 = cv2.cvtColor(img2_r, cv2.COLOR_BGR2GRAY)

        if apply_gamma:
            pre1 = apply_gamma(pre1, gamma_val)
            pre2 = apply_gamma(pre2, gamma_val)

        if apply_gaussian:
            pre1 = cv2.GaussianBlur(pre1, (3,3), 0)
            pre2 = cv2.GaussianBlur(pre2, (3,3), 0)

        # Save original grayscale and CLAHE-processed images
        cv2.imwrite(os.path.join(out_dir, f'gray_{i+1}.png'), gray1)
        cv2.imwrite(os.path.join(out_dir, f'clahe_{i+1}.png'), pre1)
        if i == len(grays) - 2:  # Save for the last image
            cv2.imwrite(os.path.join(out_dir, f'gray_{i+2}.png'), gray2)
            cv2.imwrite(os.path.join(out_dir, f'clahe_{i+2}.png'), pre2)

        # Feature detection on pre1
        p0 = cv2.goodFeaturesToTrack(pre1, mask=None, **feature_params)
        if p0 is None or len(p0) == 0:
            print(f"No features detected in preprocessed image {i+1}.")
            continue
        p0 = cv2.cornerSubPix(pre1, p0, (10,10), (-1,-1),
                              (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                               10, 0.03))

        # Calculate optical flow on preprocessed images
        p1, st, _ = cv2.calcOpticalFlowPyrLK(pre1, pre2, p0, None, **lk_params)
        good0 = p0[st.flatten() == 1]
        good1 = p1[st.flatten() == 1]
        disp = good1 - good0

        # Compute velocity only if there are tracked points
        if len(disp) > 0:
            disp_reshaped = disp.reshape(-1, 2)
            vx, vy = np.mean(disp_reshaped, axis=0)
            mag = np.sqrt(vx * vx + vy * vy)
            print(f"Preprocessed: Mean flow velocity between image {i+1} and {i+2}: vx={vx:.2f}, vy={vy:.2f}, mag={mag:.2f} px/frame")
        else:
            print(f"Preprocessed: No good points found for optical flow between image {i+1} and {i+2}.")
            vx, vy, mag = 0.0, 0.0, 0.0

        # Debug
        if debug:
            if len(disp) > 0:
                print(f"Features used between image {i+1} and {i+2}: {len(good0)}")
                mags = np.linalg.norm(disp_reshaped, axis=1)
                plt.hist(mags, bins=30)
                plt.title(f'Flow magnitude histogram between image {i+1} and {i+2}')
                plt.savefig(os.path.join(out_dir, f'hist_flow_{i+1}_{i+2}.png'))
                plt.close()
            else:
                print(f"No features to plot histogram between image {i+1} and {i+2}.")

        # Draw arrows for preprocessed
        vis = cv2.cvtColor(pre2, cv2.COLOR_GRAY2BGR)
        for (x0, y0), (x1, y1) in zip(good0.reshape(-1, 2), good1.reshape(-1, 2)):
            cv2.arrowedLine(vis, (int(x0), int(y0)), (int(x1), int(y1)),
                            (0, 255, 0), 1, tipLength=0.3)
            cv2.circle(vis, (int(x0), int(y0)), 3, (0, 0, 255), -1)
        cv2.imwrite(os.path.join(out_dir, f'flow_arrows_{i+1}_{i+2}.png'), vis)

    print(f"Outputs saved in {out_dir}")

if __name__ == '__main__':
    main()