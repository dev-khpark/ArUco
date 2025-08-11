using UnityEngine;

[CreateAssetMenu(fileName = "New Tangram Piece", menuName = "Tangram/Tangram Piece")]
public class TangramPieceData : ScriptableObject
{
    [Header("Basic Info")]
    public string pieceName;
    public Mesh mesh;
    public Material material;
    
    [Header("Transform")]
    public Vector3 position;
    public Vector3 rotation;
    public Vector3 scale = Vector3.one;
    
    [Header("Rendering Settings")]
    public bool castShadows = true;
    public bool receiveShadows = true;
    public bool contributeGlobalIllumination = true;
    
    [Header("Light Probes")]
    public UnityEngine.Rendering.LightProbeUsage lightProbeUsage = UnityEngine.Rendering.LightProbeUsage.BlendProbes;
    public UnityEngine.Rendering.ReflectionProbeUsage reflectionProbeUsage = UnityEngine.Rendering.ReflectionProbeUsage.BlendProbes;
    
    [Header("Motion Vectors")]
    public bool useCustomMotionVector = false;
    [SerializeField] private int motionVectorModeIndex = 0; // 0: Object, 1: Camera, 2: ForceNoMotion
    
    public MotionVectorGenerationMode GetMotionVectorMode()
    {
        if (!useCustomMotionVector) 
        {
            // 기본값 사용 - Unity 버전에 관계없이 안전
            return (MotionVectorGenerationMode)0;
        }
        
        var modes = System.Enum.GetValues(typeof(MotionVectorGenerationMode));
        if (motionVectorModeIndex < modes.Length)
            return (MotionVectorGenerationMode)modes.GetValue(motionVectorModeIndex);
        
        return (MotionVectorGenerationMode)0;
    }
    
    [Header("Dynamic Occlusion")]
    public bool dynamicOcclusion = true;
    
    /// <summary>
    /// 이 ScriptableObject의 정보를 바탕으로 GameObject를 생성합니다.
    /// </summary>
    public GameObject CreateGameObject(Transform parent = null)
    {
        GameObject go = new GameObject(pieceName);
        
        if (parent != null)
            go.transform.SetParent(parent);
            
        // Transform 설정
        go.transform.localPosition = position;
        go.transform.localEulerAngles = rotation;
        go.transform.localScale = scale;
        
        // MeshFilter 추가
        if (mesh != null)
        {
            MeshFilter meshFilter = go.AddComponent<MeshFilter>();
            meshFilter.mesh = mesh;
        }
        
        // MeshRenderer 추가 및 설정
        MeshRenderer renderer = go.AddComponent<MeshRenderer>();
        if (material != null)
        {
            renderer.material = material;
        }
        
        // 렌더링 설정
        renderer.shadowCastingMode = castShadows ? 
            UnityEngine.Rendering.ShadowCastingMode.On : 
            UnityEngine.Rendering.ShadowCastingMode.Off;
        renderer.receiveShadows = receiveShadows;
        
        // 라이트 프로브 설정
        renderer.lightProbeUsage = lightProbeUsage;
        renderer.reflectionProbeUsage = reflectionProbeUsage;
        
        // 모션 벡터 설정
        if (useCustomMotionVector)
        {
            renderer.motionVectorGenerationMode = GetMotionVectorMode();
        }
        
        // Dynamic Occlusion 설정
        renderer.allowOcclusionWhenDynamic = dynamicOcclusion;
        
        return go;
    }
    
    /// <summary>
    /// 기존 GameObject의 정보를 이 ScriptableObject로 복사합니다.
    /// </summary>
    public void CopyFromGameObject(GameObject sourceObject)
    {
        pieceName = sourceObject.name;
        
        // Transform 정보 복사
        position = sourceObject.transform.localPosition;
        rotation = sourceObject.transform.localEulerAngles;
        scale = sourceObject.transform.localScale;
        
        // Mesh 정보 복사
        MeshFilter meshFilter = sourceObject.GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            mesh = meshFilter.sharedMesh;
        }
        
        // Material 정보 복사
        MeshRenderer renderer = sourceObject.GetComponent<MeshRenderer>();
        if (renderer != null)
        {
            if (renderer.sharedMaterial != null)
            {
                material = renderer.sharedMaterial;
            }
            
            // 렌더링 설정 복사
            castShadows = renderer.shadowCastingMode == UnityEngine.Rendering.ShadowCastingMode.On;
            receiveShadows = renderer.receiveShadows;
            lightProbeUsage = renderer.lightProbeUsage;
            reflectionProbeUsage = renderer.reflectionProbeUsage;
            
            // 모션 벡터 설정 복사
            var modes = System.Enum.GetValues(typeof(MotionVectorGenerationMode));
            for (int i = 0; i < modes.Length; i++)
            {
                if ((MotionVectorGenerationMode)modes.GetValue(i) == renderer.motionVectorGenerationMode)
                {
                    motionVectorModeIndex = i;
                    useCustomMotionVector = true;
                    break;
                }
            }
            
            dynamicOcclusion = renderer.allowOcclusionWhenDynamic;
        }
    }
}
