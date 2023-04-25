using UnityEngine;
using UnityEngine.Rendering;
using System.Collections;
using System.IO;
using UnityEditor;
// @TODO:
// . support custom color wheels in optical flow via lookup textures
// . support custom depth encoding
// . support multiple overlay cameras
// . tests
// . better example scene(s)

// @KNOWN ISSUES
// . Motion Vectors can produce incorrect results in Unity 5.5.f3 when
//      1) during the first rendering frame
//      2) rendering several cameras with different aspect ratios - vectors do stretch to the sides of the screen

[RequireComponent(typeof(Camera))]
public class ImageSynthesis : MonoBehaviour
{   
    public float publisheFreq = 0.05f;
    public float deltaTime = 0.05f;
    public float timeElapsed1 = 0.0f;
    public bool Start_Saving = false;

    private float f = 1.0f;
    private int Pwdith = 1920;
    private int Pheight = 1080;
 
    [Header("Shader Setup")]
    public Shader uberReplacementShader;
    public Shader opticalFlowShader;
    public float opticalFlowSensitivity = 1.0f;

    [Header("Save Image Capture")]
    public bool saveImage = true;
    public bool saveIdSegmentation = true;
    public bool saveLayerSegmentation = true;
    public bool saveDepth = true;
    public bool saveNormals = true;
    public bool saveOpticalFlow;
    public string filepath = "/share/erthos/unity_data";
    public string filename = "img";
    public int counter = 0;

    // pass configuration
    private CapturePass[] capturePasses = new CapturePass[] {
        new CapturePass() { name = "_rgb" },
        new CapturePass() { name = "_id", supportsAntialiasing = false },
        new CapturePass() { name = "_layer", supportsAntialiasing = false },
        new CapturePass() { name = "_depth" },
        new CapturePass() { name = "_normals" },
        new CapturePass() { name = "_flow", supportsAntialiasing = false, needsRescale = true } // (see issue with Motion Vectors in @KNOWN ISSUES)
    };

    struct CapturePass
    {
        // configuration
        public string name;
        public bool supportsAntialiasing;
        public bool needsRescale;
        public CapturePass(string name_) { name = name_; supportsAntialiasing = true; needsRescale = false; camera = null; }

        // impl
        public Camera camera;
    };

    // cached materials
    private Material opticalFlowMaterial;

    void Start()
    {   

        // default fallbacks, if shaders are unspecified
        if (!uberReplacementShader)
            uberReplacementShader = Shader.Find("Hidden/UberReplacement");

        if (!opticalFlowShader)
            opticalFlowShader = Shader.Find("Hidden/OpticalFlow");

        // use real camera to capture final image
        capturePasses[0].camera = GetComponent<Camera>();
        for (int q = 1; q < capturePasses.Length; q++)
            capturePasses[q].camera  = CreateHiddenCamera(capturePasses[q].name);
            // capturePasses[q].camera = changeCameraParam(cam_);

            // capturePasses[q].camera = changeCameraParam(capturePasses[q].camera); 
        OnCameraChange();
        OnSceneChange();
    }

    void Update(){
        timeElapsed1 += Time.deltaTime;        
        if (timeElapsed1 > publisheFreq){
            if (Start_Saving){
                Debug.Log(filename);
                filename = string.Format("img_{0:00000}.png",counter);
                Save(filename,-1,-1,filepath);
                counter +=  1;
            }
            
            timeElapsed1 = 0;
        }
    }
    
    public Camera changeCameraParam(Camera c_camera){
        // Simulated camera paramters
        // image_width: 1920
        // image_height: 1080
        // camera_name: camera
        // camera_matrix:
        // rows: 3
        // cols: 3
        // data: [1675.59499,    0.     ,  958.75841,
        //             0.     , 1677.80857,  553.95201,
        //             0.     ,    0.     ,    1.     ]
        // distortion_model: plumb_bob
        // distortion_coefficients:
        // rows: 1
        // cols: 5
        // data: [0.172954, -0.347711, 0.002935, -0.001399, 0.000000]
        // rectification_matrix:
        // rows: 3
        // cols: 3
        // data: [1., 0., 0.,
        //         0., 1., 0.,
        //         0., 0., 1.]
        // projection_matrix:
        // rows: 3
        // cols: 4
        // data: [1709.29797,    0.     ,  956.41779,    0.     ,
        //             0.     , 1715.13635,  556.34332,    0.     ,
        //             0.     ,    0.     ,    1.     ,    0.     ]
        float ax, ay, sizeX, sizeY;
        float x0, y0, shiftX, shiftY;
        int width, height;
 
 
        ax = 1675.59499f;
        ay = 1677.80857f;
        x0 = 958.75841f;
        y0 = 553.95201f;
 
        width =  Pwdith;
        height = Pheight;       

        sizeX = f * width / ax;
        sizeY = f * height / ay;
 
        PlayerSettings.defaultScreenWidth = width;
        PlayerSettings.defaultScreenHeight = height;
 
        shiftX = -(x0 - width / 2.0f) / width;
        shiftY = (y0 - height / 2.0f) / height;
 
        c_camera.sensorSize = new Vector2(sizeX, sizeY);     // in mm, mx = 1000/x, my = 1000/y
        c_camera.focalLength = f;                            // in mm, ax = f * mx, ay = f * my
        c_camera.lensShift = new Vector2(shiftX, shiftY);    // W/2,H/w for (0,0), 1.0 shift in full W/H in image plane
        return c_camera;
    }

    
    void LateUpdate()
    {
#if UNITY_EDITOR
        if (DetectPotentialSceneChangeInEditor())
            OnSceneChange();
#endif // UNITY_EDITOR

        // @TODO: detect if camera properties actually changed
        OnCameraChange();
    }

    private Camera CreateHiddenCamera(string name)
    {
        var go = new GameObject(name, typeof(Camera));
        go.hideFlags = HideFlags.HideAndDontSave;
        go.transform.parent = transform;

        var newCamera = changeCameraParam(go.GetComponent<Camera>());
        
        return newCamera;
    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode)
    {
        SetupCameraWithReplacementShader(cam, shader, mode, Color.black);
    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode, Color clearColor)
    {
        var cb = new CommandBuffer();
        cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
        cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
        cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
        cam.SetReplacementShader(shader, "");
        cam.backgroundColor = clearColor;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.allowHDR = false;
        cam.allowMSAA = false;
    }

    static private void SetupCameraWithPostShader(Camera cam, Material material, DepthTextureMode depthTextureMode = DepthTextureMode.None)
    {
        var cb = new CommandBuffer();
        cb.Blit(null, BuiltinRenderTextureType.CurrentActive, material);
        cam.AddCommandBuffer(CameraEvent.AfterEverything, cb);
        cam.depthTextureMode = depthTextureMode;
    }

    public enum ReplacementMode
    {
        ObjectId = 0,
        CatergoryId = 1,
        DepthCompressed = 2,
        DepthMultichannel = 3,
        Normals = 4
    };

    public void OnCameraChange()
    {
        int targetDisplay = 1;
        var mainCamera = GetComponent<Camera>();
        
        foreach (var pass in capturePasses)
        {
            if (pass.camera == mainCamera)
                continue;

            // cleanup capturing camera
            pass.camera.RemoveAllCommandBuffers();

            // copy all "main" camera parameters into capturing camera
            pass.camera.CopyFrom(mainCamera);

            // set targetDisplay here since it gets overriden by CopyFrom()
            pass.camera.targetDisplay = targetDisplay++;
        }

        // cache materials and setup material properties
        if (!opticalFlowMaterial || opticalFlowMaterial.shader != opticalFlowShader)
            opticalFlowMaterial = new Material(opticalFlowShader);
        opticalFlowMaterial.SetFloat("_Sensitivity", opticalFlowSensitivity);

        // setup command buffers and replacement shaders
        SetupCameraWithReplacementShader(capturePasses[1].camera, uberReplacementShader, ReplacementMode.ObjectId);
        SetupCameraWithReplacementShader(capturePasses[2].camera, uberReplacementShader, ReplacementMode.CatergoryId);
        SetupCameraWithReplacementShader(capturePasses[3].camera, uberReplacementShader, ReplacementMode.DepthCompressed, Color.white);
        SetupCameraWithReplacementShader(capturePasses[4].camera, uberReplacementShader, ReplacementMode.Normals);
        SetupCameraWithPostShader(capturePasses[5].camera, opticalFlowMaterial, DepthTextureMode.Depth | DepthTextureMode.MotionVectors);
    }


    public void OnSceneChange()
    {
        var renderers = Object.FindObjectsOfType<Renderer>();
        var mpb = new MaterialPropertyBlock();
        foreach (var r in renderers)
        {
            var id = r.gameObject.GetInstanceID();
            var layer = r.gameObject.layer;
            var tag = r.gameObject.tag;

            mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
            mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
            r.SetPropertyBlock(mpb);
        }
    }

    public void Save(string filename, int width = -1, int height = -1, string path = "")
    {
        if (width <= 0 || height <= 0)
        {
            // width = Screen.width;
            // height = Screen.height;
            Screen.SetResolution(Pwdith, Pheight, true);
            width = Pwdith;
            height = Pheight;
        }

        var filenameExtension = System.IO.Path.GetExtension(filename);
        if (filenameExtension == "")
            filenameExtension = ".png";
        var filenameWithoutExtension = Path.GetFileNameWithoutExtension(filename);

        var pathWithoutExtension = Path.Combine(path, filenameWithoutExtension);

        // execute as coroutine to wait for the EndOfFrame before starting capture
        StartCoroutine(
            WaitForEndOfFrameAndSave(pathWithoutExtension, filenameExtension, width, height));
    }

    private IEnumerator WaitForEndOfFrameAndSave(string filenameWithoutExtension, string filenameExtension, int width, int height)
    {
        yield return new WaitForEndOfFrame();
        Save(filenameWithoutExtension, filenameExtension, width, height);
    }

    private void Save(string filenameWithoutExtension, string filenameExtension, int width, int height)
    {
        foreach (var pass in capturePasses)
        {
            // Perform a check to make sure that the capture pass should be saved
            if (
                (pass.name == "_rgb" && saveImage) ||
                (pass.name == "_id" && saveIdSegmentation) ||
                (pass.name == "_layer" && saveLayerSegmentation) ||
                (pass.name == "_depth" && saveDepth) ||
                (pass.name == "_normals" && saveNormals) ||
                (pass.name == "_flow" && saveOpticalFlow)
            )
            {
                Save(pass.camera, filenameWithoutExtension + pass.name + filenameExtension, pass.name,  width, height, pass.supportsAntialiasing, pass.needsRescale);
            }
        }
    }

    private void Save(Camera cam, string filename, string name,  int width, int height, bool supportsAntialiasing, bool needsRescale)
    {
        
        var mainCamera = GetComponent<Camera>();
        var depth = 24;
        var format = RenderTextureFormat.Default;
        var readWrite = RenderTextureReadWrite.Default;
        var antiAliasing = (supportsAntialiasing) ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

        var finalRT =
            RenderTexture.GetTemporary(width, height, depth, format, readWrite, antiAliasing);
        var renderRT = (!needsRescale) ? finalRT :
            RenderTexture.GetTemporary(mainCamera.pixelWidth, mainCamera.pixelHeight, depth, format, readWrite, antiAliasing);
        var tex = new Texture2D(width, height, TextureFormat.RGB24, false);

        var prevActiveRT = RenderTexture.active;
        var prevCameraRT = cam.targetTexture;

        // render to offscreen texture (readonly from CPU side)
        RenderTexture.active = renderRT;
        cam.targetTexture = renderRT;

        cam.Render();

        if (needsRescale)
        {
            // blit to rescale (see issue with Motion Vectors in @KNOWN ISSUES)
            RenderTexture.active = finalRT;
            Graphics.Blit(renderRT, finalRT);
            RenderTexture.ReleaseTemporary(renderRT);
        }

        // read offsreen texture contents into the CPU readable texture
        tex.ReadPixels(new Rect(0, 0, tex.width, tex.height), 0, 0);
        tex.Apply();
        
        var bytes = tex.EncodeToPNG();
        File.WriteAllBytes(filename, bytes);

        // restore state and cleanup
        cam.targetTexture = prevCameraRT;
        RenderTexture.active = prevActiveRT;

        Object.Destroy(tex);
        RenderTexture.ReleaseTemporary(finalRT);
    }

#if UNITY_EDITOR
    private GameObject lastSelectedGO;
    private int lastSelectedGOLayer = -1;
    private string lastSelectedGOTag = "unknown";
    private bool DetectPotentialSceneChangeInEditor()
    {
        bool change = false;
        // there is no callback in Unity Editor to automatically detect changes in scene objects
        // as a workaround lets track selected objects and check, if properties that are 
        // interesting for us (layer or tag) did not change since the last frame
        if (UnityEditor.Selection.transforms.Length > 1)
        {
            // multiple objects are selected, all bets are off!
            // we have to assume these objects are being edited
            change = true;
            lastSelectedGO = null;
        }
        else if (UnityEditor.Selection.activeGameObject)
        {
            var go = UnityEditor.Selection.activeGameObject;
            // check if layer or tag of a selected object have changed since the last frame
            var potentialChangeHappened = lastSelectedGOLayer != go.layer || lastSelectedGOTag != go.tag;
            if (go == lastSelectedGO && potentialChangeHappened)
                change = true;

            lastSelectedGO = go;
            lastSelectedGOLayer = go.layer;
            lastSelectedGOTag = go.tag;
        }

        return change;
    }
#endif // UNITY_EDITOR
   
}
