using UnityEngine;
using System.Collections.Generic;

[CreateAssetMenu(fileName = "New Tangram Diagram", menuName = "Tangram/Tangram Diagram")]
public class TangramDiagramData : ScriptableObject
{
    [Header("Diagram Info")]
    public string diagramName;
    [TextArea(3, 5)]
    public string description;
    
    [Header("Pieces")]
    public List<TangramPieceData> pieces = new List<TangramPieceData>();
    
    [Header("Global Settings")]
    public Vector3 rootPosition = Vector3.zero;
    public Vector3 rootRotation = Vector3.zero;
    public Vector3 rootScale = Vector3.one;
    
    /// <summary>
    /// 전체 다이어그램을 생성합니다.
    /// </summary>
    public GameObject CreateDiagram(Transform parent = null)
    {
        GameObject rootObject = new GameObject(diagramName);
        
        if (parent != null)
            rootObject.transform.SetParent(parent);
            
        // 루트 Transform 설정
        rootObject.transform.localPosition = rootPosition;
        rootObject.transform.localEulerAngles = rootRotation;
        rootObject.transform.localScale = rootScale;
        
        // 각 조각들 생성
        foreach (TangramPieceData piece in pieces)
        {
            if (piece != null)
            {
                GameObject pieceObject = piece.CreateGameObject(rootObject.transform);
            }
        }
        
        return rootObject;
    }
    
    /// <summary>
    /// 기존 GameObject 계층구조에서 다이어그램 정보를 추출합니다.
    /// </summary>
    public void ExtractFromGameObject(GameObject rootObject)
    {
        diagramName = rootObject.name;
        
        // 루트 Transform 정보 저장
        rootPosition = rootObject.transform.localPosition;
        rootRotation = rootObject.transform.localEulerAngles;
        rootScale = rootObject.transform.localScale;
        
        // 기존 pieces 리스트 클리어
        pieces.Clear();
        
        // 자식 오브젝트들을 순회하며 TangramPiece 생성
        for (int i = 0; i < rootObject.transform.childCount; i++)
        {
            Transform child = rootObject.transform.GetChild(i);
            GameObject childObject = child.gameObject;
            
            // MeshFilter와 MeshRenderer가 있는 오브젝트만 처리
            if (childObject.GetComponent<MeshFilter>() != null && 
                childObject.GetComponent<MeshRenderer>() != null)
            {
                TangramPieceData newPiece = CreateInstance<TangramPieceData>();
                newPiece.CopyFromGameObject(childObject);
                newPiece.name = childObject.name;
                
                pieces.Add(newPiece);
            }
        }
    }
    
    /// <summary>
    /// 특정 조각을 찾습니다.
    /// </summary>
    public TangramPieceData FindPiece(string pieceName)
    {
        return pieces.Find(piece => piece.pieceName == pieceName);
    }
    
    /// <summary>
    /// 조각을 추가합니다.
    /// </summary>
    public void AddPiece(TangramPieceData piece)
    {
        if (piece != null && !pieces.Contains(piece))
        {
            pieces.Add(piece);
        }
    }
    
    /// <summary>
    /// 조각을 제거합니다.
    /// </summary>
    public void RemovePiece(TangramPieceData piece)
    {
        if (pieces.Contains(piece))
        {
            pieces.Remove(piece);
        }
    }
    
    /// <summary>
    /// 모든 조각의 이름을 반환합니다.
    /// </summary>
    public string[] GetPieceNames()
    {
        string[] names = new string[pieces.Count];
        for (int i = 0; i < pieces.Count; i++)
        {
            names[i] = pieces[i] != null ? pieces[i].pieceName : "None";
        }
        return names;
    }
}