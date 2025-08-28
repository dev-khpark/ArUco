using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

#if UNITY_ANDROID
using UnityEngine.Android;
#endif

using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ArucoModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.ObjdetectModule;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.ImgprocModule;

public class ArUcoMobileTracker : MonoBehaviour
{
    [Header("Tangram Matching")]
    [Tooltip("Optional: Assign to enable matching detected shapes to the TangramDiagram children.")]
    public TangramMatcher tangramMatcher;
    [Tooltip("Camera transform used to convert ArUco camera-space tvecs to Unity world positions. If null, falls back to main camera.")]
    public Transform arUcoCameraTransform;

    [Header("UI Settings")]
    public RawImage rawImageDisplay;  // UI에 보여줄 RawImage
    
    [Header("ArUco Settings")]
    public float markerLength = 0.05f; // 마커 한 변의 실제 길이 (단위: 미터)
    [Tooltip("Draw an orientation arrow directly onto the camera image for each detected marker (projected +X axis).")]
    public bool drawMarkerOrientationOnImage = true;
    [Tooltip("Arrow length as a ratio of markerLength.")]
    [Range(0.2f, 2.0f)] public float arrowLengthRatio = 0.8f;
    [Tooltip("Arrow thickness in pixels on the image.")]
    public int arrowThicknessPx = 2;
    public Scalar arrowColorBgr = new Scalar(0, 255, 0, 255); // green
    
    [Header("Camera Settings")]
    public bool useFrontCamera = false; // true: 전면 카메라, false: 후면 카메라
    [Tooltip("카메라 해상도 폭")]
    public int cameraWidth = 1920;
    [Tooltip("카메라 해상도 높이")]
    public int cameraHeight = 1080;
    [Space]
    [Tooltip("게임 실행 중 카메라 변경을 위한 버튼 (테스트용)")]
    public bool switchCameraInRuntime = false;

    private WebCamTexture webcamTexture;
    private Texture2D texture;
    private Mat camMat;
    private Dictionary dictionary;
    private DetectorParameters detectorParams;
    private ArucoDetector arucoDetector;
    
    // 카메라 관리용 변수들
    private bool lastUseFrontCamera;
    private bool lastSwitchCameraInRuntime;

    private Mat ids;
    private List<Mat> corners;
    private Mat rvecs;
    private Mat tvecs;

    // 예시용 기본 카메라 행렬 및 왜곡 계수
    private Mat camMatrix;
    private MatOfDouble distCoeffs;

    // -------- Stability buffer (10-frame time shifting) --------
    private struct BufferedSample
    {
        public int frame;
        public int arucoId;
        public TangramMatcher.TangramShapeType shapeType;
        public Vector3 worldPos;
        public Quaternion worldRot;
        public bool isCorner;
    }

    [Tooltip("Number of frames for stability time-shifting window.")]
    public int stabilityWindowFrames = 10;

    // Buffer per ArUco ID
    private readonly Dictionary<int, List<BufferedSample>> bufferById = new Dictionary<int, List<BufferedSample>>();

    private void AddToBuffer(int arucoId, TangramMatcher.TangramShapeType type, Vector3 worldPos, Quaternion worldRot, bool isCorner)
    {
        if (!bufferById.TryGetValue(arucoId, out var list))
        {
            list = new List<BufferedSample>();
            bufferById[arucoId] = list;
        }
        list.Add(new BufferedSample
        {
            frame = Time.frameCount,
            arucoId = arucoId,
            shapeType = type,
            worldPos = worldPos,
            worldRot = worldRot,
            isCorner = isCorner
        });
        // Prune old frames beyond window
        int minFrame = Time.frameCount - Mathf.Max(1, stabilityWindowFrames) + 1;
        int idx = 0;
        while (idx < list.Count)
        {
            if (list[idx].frame >= minFrame) break;
            idx++;
        }
        if (idx > 0) list.RemoveRange(0, idx);
    }

    private List<TangramMatcher.DetectedShape> BuildStableDetectionsFromBuffer()
    {
        var stable = new List<TangramMatcher.DetectedShape>();
        int minFrame = Time.frameCount - Mathf.Max(1, stabilityWindowFrames) + 1;
        foreach (var kv in bufferById)
        {
            var samples = kv.Value;
            // Keep only samples within window
            var window = new List<BufferedSample>();
            for (int i = 0; i < samples.Count; i++)
            {
                if (samples[i].frame >= minFrame) window.Add(samples[i]);
            }
            if (window.Count == 0) continue;

            // Compute per-axis medians
            float Median(List<float> arr)
            {
                arr.Sort();
                int n = arr.Count;
                if (n == 0) return 0f;
                if ((n & 1) == 1) return arr[n / 2];
                return 0.5f * (arr[n / 2 - 1] + arr[n / 2]);
            }

            var xs = new List<float>(window.Count);
            var ys = new List<float>(window.Count);
            var zs = new List<float>(window.Count);
            for (int i = 0; i < window.Count; i++)
            {
                xs.Add(window[i].worldPos.x);
                ys.Add(window[i].worldPos.y);
                zs.Add(window[i].worldPos.z);
            }
            Vector3 median = new Vector3(Median(xs), Median(ys), Median(zs));

            // Choose medoid (closest to median position)
            int bestIdx = 0; float bestDist = float.PositiveInfinity;
            for (int i = 0; i < window.Count; i++)
            {
                float d = Vector3.SqrMagnitude(window[i].worldPos - median);
                if (d < bestDist)
                {
                    bestDist = d; bestIdx = i;
                }
            }
            var best = window[bestIdx];
            stable.Add(new TangramMatcher.DetectedShape
            {
                arucoId = best.arucoId,
                shapeType = best.shapeType,
                worldPosition = best.worldPos,
                worldRotation = best.worldRot,
                confidence = 1f,
                isCorner = best.isCorner
            });
        }
        return stable;
    }

    private bool TryMapArucoIdToShape(int arucoId, out TangramMatcher.TangramShapeType shapeType)
    {
        switch (arucoId)
        {
            // 0, 1: Large Triangle
            case 0: shapeType = TangramMatcher.TangramShapeType.LargeTriangle; return true;
            case 1: shapeType = TangramMatcher.TangramShapeType.LargeTriangle; return true;
            // 2: Medium Triangle
            case 2: shapeType = TangramMatcher.TangramShapeType.MediumTriangle; return true;
            // 3, 4: Small Triangle
            case 3: shapeType = TangramMatcher.TangramShapeType.SmallTriangle; return true;
            case 4: shapeType = TangramMatcher.TangramShapeType.SmallTriangle; return true;
            // 5: Square
            case 5: shapeType = TangramMatcher.TangramShapeType.Square; return true;
            // 6, 7: Parallel (Parallelogram)
            case 6: shapeType = TangramMatcher.TangramShapeType.Parallelogram; return true;
            case 7: shapeType = TangramMatcher.TangramShapeType.Parallelogram; return true;
            default:
                shapeType = default;
                return false;
        }
    }

    void Start()
    {
        RequestCameraPermission();

        // 초기화 - ArUco 딕셔너리와 감지 파라미터 설정
        dictionary = Objdetect.getPredefinedDictionary(Objdetect.DICT_4X4_50);
        detectorParams = new DetectorParameters();
        arucoDetector = new ArucoDetector(dictionary, detectorParams);

        ids = new Mat();
        corners = new List<Mat>();
        rvecs = new Mat();
        tvecs = new Mat();

        // 카메라 행렬과 왜곡 계수 예시값 (기본값, 정확한 위치 추정 원할 경우 캘리브레이션 필요)
        double fx = 800, fy = 800, cx = 320, cy = 240;
        camMatrix = new Mat(3, 3, CvType.CV_64F);
        camMatrix.put(0, 0, fx); camMatrix.put(0, 1, 0);  camMatrix.put(0, 2, cx);
        camMatrix.put(1, 0, 0);  camMatrix.put(1, 1, fy); camMatrix.put(1, 2, cy);
        camMatrix.put(2, 0, 0);  camMatrix.put(2, 1, 0);  camMatrix.put(2, 2, 1);

        distCoeffs = new MatOfDouble(0, 0, 0, 0); // 왜곡 계수 없음 처리

        // 카메라 시작
        lastUseFrontCamera = useFrontCamera;
        lastSwitchCameraInRuntime = switchCameraInRuntime;
        StartWebCam();
    }

    void RequestCameraPermission()
    {
#if UNITY_ANDROID
        if (!Permission.HasUserAuthorizedPermission(Permission.Camera))
        {
            Permission.RequestUserPermission(Permission.Camera);
        }
#endif
    }

    void StartWebCam()
    {
        WebCamDevice[] devices = WebCamTexture.devices;
        if (devices.Length > 0)
        {
            string selectedCameraName = GetSelectedCamera(devices);
            
            if (!string.IsNullOrEmpty(selectedCameraName))
            {
                webcamTexture = new WebCamTexture(selectedCameraName, cameraWidth, cameraHeight);

                
                webcamTexture.Play();

                rawImageDisplay.texture = webcamTexture;
                rawImageDisplay.material.mainTexture = webcamTexture;
                if (useFrontCamera){

                    rawImageDisplay.rectTransform.localEulerAngles = new Vector3(0, 0, -webcamTexture.videoRotationAngle);
                    rawImageDisplay.rectTransform.localScale = new Vector3(1, -1, 1);
                }
                

                // Camera successfully started, print debug information in English
                Debug.Log($"Camera started: {selectedCameraName} ({cameraWidth}x{cameraHeight})");
            }
            else
            {
                Debug.LogError("Could not find the selected camera!");
            }
        }
        else
        {
            Debug.LogError("No available camera devices found!");
        }
    }

    string GetSelectedCamera(WebCamDevice[] devices)
    {
        // 사용 가능한 모든 카메라 로그 출력
            // Print the number of available cameras in English
            Debug.Log($"Number of available cameras: {devices.Length}");
            for (int i = 0; i < devices.Length; i++)
            {
                Debug.Log($"Camera {i}: {devices[i].name}, isFrontFacing: {devices[i].isFrontFacing}");
            }

            // 원하는 카메라 찾기
        for (int i = 0; i < devices.Length; i++)
        {
            if (devices[i].isFrontFacing == useFrontCamera)
            {
                return devices[i].name;
            }
        }

        // If the desired camera is not found, use the first available camera.
        Debug.LogWarning($"Could not find the desired camera (front: {useFrontCamera}). Using the first available camera instead.");
        return devices[0].name;
    }

    void Update()
    {
        // Detect runtime camera change
        if (lastUseFrontCamera != useFrontCamera || 
            (switchCameraInRuntime && !lastSwitchCameraInRuntime))
        {
            lastUseFrontCamera = useFrontCamera;
            lastSwitchCameraInRuntime = switchCameraInRuntime;
            
            Debug.Log($"Camera change detected: Using front camera = {useFrontCamera}");
            SwitchCamera();
        }
        
        if (webcamTexture == null || !webcamTexture.isPlaying || !webcamTexture.didUpdateThisFrame)
            return;

        // 카메라 프레임을 Mat으로 변환
        if (camMat == null || camMat.width() != webcamTexture.width || camMat.height() != webcamTexture.height)
        {
            camMat = new Mat(webcamTexture.height, webcamTexture.width, CvType.CV_8UC3);
        }
    

        Utils.webCamTextureToMat(webcamTexture, camMat);

        //Flip camMat
        if (useFrontCamera)
        {
            Core.flip(camMat, camMat, 1); // 1 = 좌우 반전
        }
     

        // ArUco 마커 감지
        arucoDetector.detectMarkers(camMat, corners, ids);
            
        if (ids.total() > 0)
        {
            Objdetect.drawDetectedMarkers(camMat, corners, ids);

            // 위치 및 회전 추정
            Aruco.estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < ids.total(); i++)
            {
                // 포즈 정보 출력
                int id = (int)ids.get(i, 0)[0];
                double[] t = tvecs.get(i, 0); // (x, y, z)
                double[] r = rvecs.get(i, 0); // 회전 벡터

                // Debug output in English for detected ArUco marker position and rotation vector
                Debug.Log($"[Frame {Time.frameCount}] [ArUco ID: {id}] Position (meters): X={t[0]:F2}, Y={t[1]:F2}, Z={t[2]:F2}");
                Debug.Log($"[Frame {Time.frameCount}] [ArUco ID: {id}] Rotation Vector: X={r[0]:F2}, Y={r[1]:F2}, Z={r[2]:F2}");
                
                // Calculate planar coordinates and angle using TangramMatcher coordinate system
                if (tangramMatcher != null)
                {
                    Vector3 posCamera = new Vector3((float)t[0], (float)(-t[1]), (float)t[2]);
                    Vector3 worldPos = (arUcoCameraTransform != null ? arUcoCameraTransform : Camera.main?.transform)?.TransformPoint(posCamera) ?? Vector3.zero;
                    
                    // Compute world rotation from rvec
                    Quaternion worldRot = Quaternion.identity;
                    try
                    {
                        using (Mat rvec = new Mat(3, 1, CvType.CV_64F))
                        using (Mat rmat = new Mat(3, 3, CvType.CV_64F))
                        {
                            rvec.put(0, 0, r[0]);
                            rvec.put(1, 0, r[1]);
                            rvec.put(2, 0, r[2]);
                            Calib3d.Rodrigues(rvec, rmat);
                            Matrix4x4 m = Matrix4x4.identity;
                            m.m00 = (float)rmat.get(0, 0)[0]; m.m01 = (float)rmat.get(0, 1)[0]; m.m02 = (float)rmat.get(0, 2)[0];
                            m.m10 = (float)rmat.get(1, 0)[0]; m.m11 = (float)rmat.get(1, 1)[0]; m.m12 = (float)rmat.get(1, 2)[0];
                            m.m20 = (float)rmat.get(2, 0)[0]; m.m21 = (float)rmat.get(2, 1)[0]; m.m22 = (float)rmat.get(2, 2)[0];
                            Matrix4x4 flipY = Matrix4x4.Scale(new Vector3(1, -1, 1));
                            m = flipY * m * flipY;
                            var camTr = arUcoCameraTransform != null ? arUcoCameraTransform : Camera.main?.transform;
                            if (camTr != null)
                                worldRot = camTr.rotation * Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
                        }
                    }
                    catch (System.Exception) { }
                    
                    // Use TangramMatcher coordinate system for planar projection
                    Vector3 planeN = tangramMatcher.useXYPlane ? Vector3.forward : (tangramMatcher.tangramDiagramRoot != null ? tangramMatcher.tangramDiagramRoot.up : Vector3.up);
                    Vector3 planeOrigin = tangramMatcher.tangramDiagramRoot != null ? tangramMatcher.tangramDiagramRoot.position : Vector3.zero;
                    Vector3 axisU = Vector3.Cross(planeN, Vector3.up);
                    if (axisU.sqrMagnitude < 1e-6f) axisU = Vector3.Cross(planeN, Vector3.right);
                    axisU.Normalize();
                    Vector3 axisV = Vector3.Cross(planeN, axisU).normalized;
                    
                    // Project world position onto plane
                    Vector3 projected = worldPos - Vector3.Dot(worldPos - planeOrigin, planeN) * planeN;
                    Vector3 rel = projected - planeOrigin;
                    float projX = Vector3.Dot(rel, axisU);
                    float projY = Vector3.Dot(rel, axisV);
                    
                    // Calculate marker orientation angle on plane
                    Vector3 markerDir = Vector3.ProjectOnPlane(worldRot * Vector3.right, planeN).normalized;
                    float planeAngle = 0f;
                    if (markerDir.sqrMagnitude > 1e-6f)
                    {
                        planeAngle = Vector3.SignedAngle(axisU, markerDir, planeN);
                        while (planeAngle > 180f) planeAngle -= 360f;
                        while (planeAngle < -180f) planeAngle += 360f;
                    }
                    
                    Debug.Log($"[Frame {Time.frameCount}] [ArUco ID: {id}] Planar Coord: ({projX:F4}, {projY:F4}) | Plane Angle: {planeAngle:F1}°");
                }

                // 필요 시, rvec -> 회전 행렬 -> Quaternion 변환 후 Unity 오브젝트에 적용 가능
            }

            // Forward detections to TangramMatcher for comparison/highlighting
            if (tangramMatcher != null)
            {
                var camTr = arUcoCameraTransform != null ? arUcoCameraTransform : (Camera.main != null ? Camera.main.transform : null);
                if (camTr == null)
                {
                    Debug.LogWarning("ArUcoMobileTracker: No camera transform available for world conversion; skipping Tangram matching this frame.");
                }
                else
                {
                    List<TangramMatcher.DetectedShape> detections = new List<TangramMatcher.DetectedShape>();
                    int count = (int)ids.total();
                    for (int i = 0; i < count; i++)
                    {
                        int arucoId = (int)ids.get(i, 0)[0];
                        bool isCorner = tangramMatcher != null && (
                            arucoId == tangramMatcher.planeCornerIdTL ||
                            arucoId == tangramMatcher.planeCornerIdTR ||
                            arucoId == tangramMatcher.planeCornerIdBR ||
                            arucoId == tangramMatcher.planeCornerIdBL);

                        bool mapped = TryMapArucoIdToShape(arucoId, out TangramMatcher.TangramShapeType type);
                        if (!mapped && !isCorner)
                            continue; // ignore unknown non-corner ids
                        if (!mapped && isCorner)
                            type = TangramMatcher.TangramShapeType.Square; // placeholder; not used in matching

                        double[] t = tvecs.get(i, 0);
                        Vector3 posCamera = new Vector3((float)t[0], (float)(-t[1]), (float)t[2]);
                        Vector3 worldPos = camTr.TransformPoint(posCamera);

                        // Compute world rotation from rvec for Euler display
                        Quaternion worldRot = Quaternion.identity;
                        try
                        {
                            if (rvecs != null && rvecs.total() > i)
                            {
                                double[] r = rvecs.get(i, 0);
                                using (Mat rvec = new Mat(3, 1, CvType.CV_64F))
                                using (Mat rmat = new Mat(3, 3, CvType.CV_64F))
                                {
                                    rvec.put(0, 0, r[0]);
                                    rvec.put(1, 0, r[1]);
                                    rvec.put(2, 0, r[2]);
                                    Calib3d.Rodrigues(rvec, rmat);
                                    Matrix4x4 m = Matrix4x4.identity;
                                    m.m00 = (float)rmat.get(0, 0)[0]; m.m01 = (float)rmat.get(0, 1)[0]; m.m02 = (float)rmat.get(0, 2)[0];
                                    m.m10 = (float)rmat.get(1, 0)[0]; m.m11 = (float)rmat.get(1, 1)[0]; m.m12 = (float)rmat.get(1, 2)[0];
                                    m.m20 = (float)rmat.get(2, 0)[0]; m.m21 = (float)rmat.get(2, 1)[0]; m.m22 = (float)rmat.get(2, 2)[0];
                                    // Convert OpenCV camera coords (x right, y down, z forward) to Unity world
                                    Matrix4x4 flipY = Matrix4x4.Scale(new Vector3(1, -1, 1));
                                    m = flipY * m * flipY;
                                    worldRot = camTr.rotation * Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
                                }
                            }
                        }
                        catch (System.Exception)
                        {
                            worldRot = Quaternion.identity;
                        }

                        // Push into stability buffer
                        AddToBuffer(arucoId, type, worldPos, worldRot, isCorner);
                        // For immediate visualization, still collect raw detections this frame (optional)
                        detections.Add(new TangramMatcher.DetectedShape
                        {
                            arucoId = arucoId,
                            shapeType = type,
                            worldPosition = worldPos,
                            worldRotation = worldRot,
                            confidence = 1f,
                            isCorner = isCorner
                        });

                        if (isCorner)
                        {
                            tangramMatcher.RegisterCornerObservation(arucoId, worldPos);
                        }
                    }

                    // Build stable detections over the last N frames; prefer medoid per ID
                    var stable = BuildStableDetectionsFromBuffer();
                    // Use stable detections for matching
                    tangramMatcher.SetDetections(stable);
                }

                // Draw orientation arrows onto the camera image (so it appears exactly over the marker)
                if (drawMarkerOrientationOnImage)
                {
                    int count = (int)ids.total();
                    double arrowLen = Mathf.Max(1e-6f, markerLength * arrowLengthRatio);
                    for (int i = 0; i < count; i++)
                    {
                        // Build object points: origin and +X axis endpoint in marker coordinates
                        using (MatOfPoint3f objPts = new MatOfPoint3f(
                            new Point3(0, 0, 0),
                            new Point3(arrowLen, 0, 0)))
                        using (Mat rvec = new Mat(3, 1, CvType.CV_64F))
                        using (Mat tvec = new Mat(3, 1, CvType.CV_64F))
                        using (MatOfPoint2f imgPts = new MatOfPoint2f())
                        {
                            double[] rv = rvecs.get(i, 0);
                            double[] tv = tvecs.get(i, 0);
                            rvec.put(0, 0, rv[0]); rvec.put(1, 0, rv[1]); rvec.put(2, 0, rv[2]);
                            tvec.put(0, 0, tv[0]); tvec.put(1, 0, tv[1]); tvec.put(2, 0, tv[2]);
                            Calib3d.projectPoints(objPts, rvec, tvec, camMatrix, distCoeffs, imgPts);
                            Point[] arr = imgPts.toArray();
                            if (arr != null && arr.Length >= 2)
                            {
                                Imgproc.arrowedLine(camMat, arr[0], arr[1], arrowColorBgr, arrowThicknessPx, Imgproc.LINE_AA, 0, 0.2);
                            }
                        }
                    }
                }
            }
        }

        // 화면 출력
        if (texture == null || texture.width != camMat.width() || texture.height != camMat.height())
        {
            texture = new Texture2D(camMat.width(), camMat.height(), TextureFormat.RGBA32, false);

            

            rawImageDisplay.texture = texture;

            

        }

        Utils.matToTexture2D(camMat, texture);
    }

    void SwitchCamera()
    {
        // 기존 카메라 정지 및 정리
        if (webcamTexture != null)
        {
            webcamTexture.Stop();
            webcamTexture = null;
        }
        
        // 새 카메라 시작
        StartWebCam();
        
        // 화면 전환 시 텍스처 초기화
        if (texture != null)
        {
            DestroyImmediate(texture);
            texture = null;
        }
        
        // Mat 객체 초기화
        if (camMat != null)
        {
            camMat.Dispose();
            camMat = null;
        }
    }

    void OnDestroy()
    {
        // 리소스 정리
        if (webcamTexture != null)
        {
            webcamTexture.Stop();
            webcamTexture = null;
        }
        
        if (texture != null)
        {
            DestroyImmediate(texture);
            texture = null;
        }
        
        if (camMat != null)
        {
            camMat.Dispose();
            camMat = null;
        }
        
        if (ids != null)
        {
            ids.Dispose();
            ids = null;
        }
        
        if (rvecs != null)
        {
            rvecs.Dispose();
            rvecs = null;
        }
        
        if (tvecs != null)
        {
            tvecs.Dispose();
            tvecs = null;
        }
        
        if (camMatrix != null)
        {
            camMatrix.Dispose();
            camMatrix = null;
        }
        
        if (distCoeffs != null)
        {
            distCoeffs.Dispose();
            distCoeffs = null;
        }
    }
}