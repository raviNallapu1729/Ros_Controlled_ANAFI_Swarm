def Print_Drone_Actns(actn0, acn0_N):

	
	n_acn0 = len(actn0) 
	cn = n_acn0-1 

	print('#############################################################')
	print('  Hello, Welcome to the Anafi Swarm Control Interface!   ')
	print('############################################################# \n')
	print('Select a command from the following list: \n')
	for i in range(n_acn0):
		print(str(acn0_N[i]) + '-' + actn0[i])
	print('')
	x1 = int(input('Enter your command number:'))

	if x1<n_acn0:
		cn = acn0_N.index(x1)
	elif x1 == n_acn0:
		x1 = acn0_N[cn]
	return x1, cn