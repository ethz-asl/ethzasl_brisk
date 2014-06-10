import argparse
import ShelveWrapper as sw
import sm
import pickle 
import numpy as np

startFrame = 1400
endFrame = 2700

def buildEssentialMatrices(inputshelve, tag):
  s = sw.ShelveDb(inputshelve)
  print 'building essential matrices...'
  essentialMatrices = {}
  for index in range(startFrame, endFrame+1):
    print 'at index=', index

    mf = s[index][0]
    n = mf.numKeypoints()
    M = np.zeros((n, 3))
    S = np.zeros((n, 1))
    for i in range(n):
      kp = mf.keypoint(i)
      M[i,:] = kp.backProjection()
      S[i,0] = kp.invR()[0,0]

    essentialMatrices[index] = (M, S)

  pickle.dump(essentialMatrices, open('essentialMatrices_' + tag + '.bin', 'w'))
  print 'done!'

def extractFrameIndices(inputshelve, tag, min_deg):
  s = sw.ShelveDb(inputshelve)
  print 'extracting indices'
  indices = [startFrame]
  mfA, T_w_a = s[startFrame]  
  T_a_w = T_w_a.inverse() 
  q = min_deg
  for b in range(startFrame + 1, endFrame+1):
    print 'at b: ', b
    mfB, T_w_b = s[b]
    T_a_b = T_a_w * T_w_b
    angle = sm.rad2deg(sm.R2rph(T_a_b.C())[2])
    if angle < 0.0:
      pangle = 360.0 + angle
    else:
      pangle = angle

    if pangle >= q:
      indices.append(b)
      q += min_deg

    if q > 150.0:
      break

  if not endFrame in indices:
    indices.append(endFrame)


  pickle.dump(indices, open('indices_' + tag + '.bin', 'w'))
  print 'done!'

  return indices

def main():
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("inputshelve", type=str, help="")
    parser.add_argument("tag", type=str, help="")
    parser.add_argument("min_deg", type=str, help="")

    args = parser.parse_args()

    inputshelve = str(args.inputshelve)
    tag = str(args.tag)
    min_deg = str(args.min_deg)

    process(inputshelve, tag, min_deg)

if __name__ == '__main__':
    main()
