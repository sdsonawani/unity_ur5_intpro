using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MirrorCamera : MonoBehaviour
{
    public Camera cam;

    private void Start()
    {
        var scale = Matrix4x4.Scale(new Vector3(-1, 1, 1));
        cam.ResetWorldToCameraMatrix();
        cam.ResetProjectionMatrix();
        cam.projectionMatrix *= scale;
    }

    private void OnPreRender()
    {
        GL.invertCulling = true;
    }

    private void OnPostRender()
    {
        GL.invertCulling = false;
    }
}