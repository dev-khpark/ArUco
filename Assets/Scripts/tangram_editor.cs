#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.IO;

public class TangramEditorTools : EditorWindow
{
    private TangramDiagramData selectedDiagram;
    private GameObject selectedGameObject;
    
    [MenuItem("Tools/Tangram Tools")]
    public static void ShowWindow()
    {
        GetWindow<TangramEditorTools>("Tangram Tools");
    }
    
    private void OnGUI()
    {
        GUILayout.Label("Tangram Diagram Tools", EditorStyles.boldLabel);
        
        EditorGUILayout.Space();
        
        // GameObject to ScriptableObject 변환
        GUILayout.Label("Convert GameObject to ScriptableObject", EditorStyles.label);
        selectedGameObject = (GameObject)EditorGUILayout.ObjectField(
            "Source GameObject", selectedGameObject, typeof(GameObject), true);
            
        if (GUILayout.Button("Create Tangram Diagram Asset"))
        {
            CreateTangramDiagramFromGameObject();
        }
        
        EditorGUILayout.Space();
        
        // ScriptableObject to GameObject 생성
        GUILayout.Label("Create GameObject from ScriptableObject", EditorStyles.label);
        selectedDiagram = (TangramDiagramData)EditorGUILayout.ObjectField(
            "Tangram Diagram", selectedDiagram, typeof(TangramDiagramData), false);
            
        if (GUILayout.Button("Create GameObject"))
        {
            CreateGameObjectFromDiagram();
        }
        
        EditorGUILayout.Space();
        
        // 선택된 오브젝트에서 개별 피스 생성
        if (GUILayout.Button("Create Individual Pieces from Selected"))
        {
            CreateIndividualPieces();
        }
    }
    
    private void CreateTangramDiagramFromGameObject()
    {
        if (selectedGameObject == null)
        {
            EditorUtility.DisplayDialog("Error", "Please select a GameObject first.", "OK");
            return;
        }
        
        // TangramDiagram 에셋 생성
        TangramDiagramData diagram = CreateInstance<TangramDiagramData>();
        diagram.ExtractFromGameObject(selectedGameObject);
        
        // 저장 경로 설정
        string path = EditorUtility.SaveFilePanelInProject(
            "Save Tangram Diagram", 
            selectedGameObject.name + "_Diagram", 
            "asset", 
            "Save the tangram diagram asset");
            
        if (!string.IsNullOrEmpty(path))
        {
            AssetDatabase.CreateAsset(diagram, path);
            
            // 개별 피스들도 서브에셋으로 저장
            foreach (TangramPieceData piece in diagram.pieces)
            {
                if (piece != null)
                {
                    AssetDatabase.AddObjectToAsset(piece, diagram);
                }
            }
            
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            
            // 생성된 에셋 선택
            Selection.activeObject = diagram;
            EditorGUIUtility.PingObject(diagram);
            
            EditorUtility.DisplayDialog("Success", 
                $"Tangram Diagram '{diagram.diagramName}' created successfully!", "OK");
        }
    }
    
    private void CreateGameObjectFromDiagram()
    {
        if (selectedDiagram == null)
        {
            EditorUtility.DisplayDialog("Error", "Please select a Tangram Diagram first.", "OK");
            return;
        }
        
        GameObject createdObject = selectedDiagram.CreateDiagram();
        
        // 생성된 오브젝트를 선택
        Selection.activeGameObject = createdObject;
        
        // Scene view에서 포커스
        if (SceneView.lastActiveSceneView != null)
        {
            SceneView.lastActiveSceneView.FrameSelected();
        }
        
        EditorUtility.DisplayDialog("Success", 
            $"GameObject '{createdObject.name}' created successfully!", "OK");
    }
    
    private void CreateIndividualPieces()
    {
        GameObject[] selectedObjects = Selection.gameObjects;
        
        if (selectedObjects.Length == 0)
        {
            EditorUtility.DisplayDialog("Error", "Please select at least one GameObject.", "OK");
            return;
        }
        
        string folderPath = EditorUtility.SaveFolderPanel(
            "Select folder to save pieces", 
            "Assets", 
            "TangramPieces");
            
        if (string.IsNullOrEmpty(folderPath))
            return;
            
        // Assets 폴더 내의 상대 경로로 변환
        if (!folderPath.StartsWith(Application.dataPath))
        {
            EditorUtility.DisplayDialog("Error", "Please select a folder within the Assets directory.", "OK");
            return;
        }
        
        string relativePath = "Assets" + folderPath.Substring(Application.dataPath.Length);
        
        int createdCount = 0;
        
        foreach (GameObject obj in selectedObjects)
        {
            if (obj.GetComponent<MeshFilter>() != null && obj.GetComponent<MeshRenderer>() != null)
            {
                TangramPieceData piece = CreateInstance<TangramPieceData>();
                piece.CopyFromGameObject(obj);
                
                string assetPath = Path.Combine(relativePath, obj.name + "_Piece.asset");
                AssetDatabase.CreateAsset(piece, assetPath);
                createdCount++;
            }
        }
        
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        
        EditorUtility.DisplayDialog("Success", 
            $"{createdCount} Tangram Pieces created successfully!", "OK");
    }
}

// Context Menu 추가
public class TangramContextMenu
{
    [MenuItem("GameObject/Tangram/Create Tangram Diagram Asset", false, 0)]
    static void CreateTangramDiagramAsset(MenuCommand menuCommand)
    {
        GameObject go = menuCommand.context as GameObject;
        if (go == null) return;
        
        TangramDiagramData diagram = ScriptableObject.CreateInstance<TangramDiagramData>();
        diagram.ExtractFromGameObject(go);
        
        string path = $"Assets/{go.name}_TangramDiagram.asset";
        path = AssetDatabase.GenerateUniqueAssetPath(path);
        
        AssetDatabase.CreateAsset(diagram, path);
        
        foreach (TangramPieceData piece in diagram.pieces)
        {
            if (piece != null)
            {
                AssetDatabase.AddObjectToAsset(piece, diagram);
            }
        }
        
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        
        Selection.activeObject = diagram;
        EditorGUIUtility.PingObject(diagram);
    }
    
    [MenuItem("GameObject/Tangram/Create Tangram Diagram Asset", true)]
    static bool ValidateCreateTangramDiagramAsset()
    {
        return Selection.activeGameObject != null;
    }
}
#endif